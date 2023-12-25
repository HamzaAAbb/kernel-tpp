#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>  // copy_to/from_user()
#include <linux/gpio.h>     // GPIO
#include <linux/interrupt.h>
#include <linux/err.h>

/*
   La gestion de l'instabilite des entrees (debouncing) n'est pas disponible sous
   Raspberry.
   Ce code est ajoute pour eviter les faux positifs (qui risquent de declencher
   plusieurs IRQ pour un seul evenement).
   D'autres materiels peuvent supporter le debouncing, ce code est alors inutile
   pour eliminer ce code en extra, il suffit de commenter la macro ci-dessous
   Le code est inspire de:
   https://raspberrypi.stackexchange.com/questions/8544/gpio-interrupt-debounce

   Pour utiliser le Debounce materiel (s'il existe), il faut commenter la macro
   EN_DEBOUNCE.
*/

#define EN_DEBOUNCE

#ifdef EN_DEBOUNCE
#include <linux/jiffies.h>
extern unsigned long volatile jiffies; // variable du kernel qui mesure le nombre
                                       // d'interruptions de l'horloge depuis le demarrage du systeme (donne une "mesure"
                                       // du temps)
unsigned long old_jiffies = 0;
#endif

/* Fin du code extra pour la gestion du Debounce */

// La LED est connectee en sortie a ce GPIO
#define GPIO_21_OUT (21)

// L'interrupteur est connecte en entree a ce GPIO
#define GPIO_25_IN (25)

// GPIO_25_IN valeur (bascule) a affecter au led (0 -> eteinte, 1 -> allumee).
unsigned int led_toggle = 0;

// Utilisee pour stocker le numero d'IRQ du GPIO
unsigned int GPIO_irqNumber;

/*
   Fonction de traitement du GPIO 25. Cette fonction est
   appelee a chaque front montant.
*/
static irqreturn_t gpio_irq_handler(int irq, void *dev_id) {
    static unsigned long flags = 0;

#ifdef EN_DEBOUNCE
    unsigned long diff = jiffies - old_jiffies;
    if (diff < 200) {
        return IRQ_HANDLED;
    }
    old_jiffies = jiffies;
#endif
    // Reste du code à écrire...
}

led_toggle = (0x01 ^ led_toggle);  // on permute l’ancienne valeur
gpio_set_value(GPIO_21_OUT, led_toggle);  // on positionne le GPIO GPIO_21_OUT
pr_info("interruption survenue : GPIO_21_OUT : %d", gpio_get_value(GPIO_21_OUT));
local_irq_restore(flags);  // on reactive de nouveau les interruptions
                            // dans le CPU en cours en restaurant leur etat anterieur avant desactivation
return IRQ_HANDLED;  // on retourne que l’interruption a bien ete traitee
}

dev_t dev = 0;
static struct class *dev_class;
static struct cdev sud_cdev;
static int __init sud_driver_init(void);
static void __exit sud_driver_exit(void);

// Fonctions du pilote
static int sud_open(struct inode *inode, struct file *file);
static int sud_release(struct inode *inode, struct file *file);
static ssize_t sud_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
static ssize_t sud_write(struct file *filp, const char *buf, size_t len, loff_t *off);

// Structure file_operations
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = sud_read,
    .write = sud_write,
    .open = sud_open,
    .release = sud_release,
};

// Fonction appelee quand on ouvre le fichier peripherique
static int sud_open(struct inode *inode, struct file *file) {
    pr_info("Fichier peripherique ouvert...!!!\n");
    return 0;
}

// Fonction appelee quand on ferme le fichier peripherique
static int sud_release(struct inode *inode, struct file *file) {
    pr_info("Fichier peripherique ferme...!!!\n");
    return 0;
}

// Fonction appelee quand on lit depuis le fichier peripherique
static ssize_t sud_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    uint8_t gpio_state = 0;
    // Lecture de la valeur du GPIO
    gpio_state = gpio_get_value(GPIO_21_OUT);
    // Ecriture du resultat vers le user space
    len = 1;
    if (copy_to_user(buf, &gpio_state, len) > 0) {
        pr_err("ERREUR: Certains octets n'ont pas pu etre copies vers le user space\n");
    }
    pr_info("sud_read : GPIO_21 = %d\n", gpio_state);
    return 0;
}

/*
  Fonction appelee quand on ecrit dans le fichier peripherique
*/
static ssize_t sud_write(struct file *filp, const char __user *buf, size_t len, loff_t *off) {
    uint8_t rec_buf[10] = {0};
    if (copy_from_user(rec_buf, buf, len) > 0) {
        pr_err("ERREUR: Certains octets n’ont pas pu etre copies depuis le user space\n");
    }
    pr_info("sud_write : GPIO_21 Set = %c\n", rec_buf[0]);
    if (rec_buf[0] == '1') {
        // On met la valeur du GPIO (en sortie) a HIGH
        gpio_set_value(GPIO_21_OUT, 1);
    } else if (rec_buf[0] == '0') {
        // On met la valeur du GPIO (en sortie) a LOW
        gpio_set_value(GPIO_21_OUT, 0);
    } else {
        pr_err("Commande inconnue : merci de donner ou bien 1 ou bien 0\n");
    }
    return len;
}

/*
  Fonction d’initialisation du module
*/
static int __init sud_driver_init(void) {
    /* Allocation du nombre Majeur */
    if (alloc_chrdev_region(&dev, 0, 1, "sud_Dev") < 0) {
        pr_err("Impossible d’allouer le nombre Majeur\n");
        goto r_unreg;
    }
    pr_info("Majeur = %d Mineur = %d\n", MAJOR(dev), MINOR(dev));
    /* Creation de la structure cdev */
    cdev_init(&sud_cdev, &fops);
    /* Ajout du peripherique character au systeme */
    if (cdev_add(&sud_cdev, dev, 1) < 0) {
        pr_err("Impossible d’ajouter le peripherique au systeme\n");
        goto r_del;
    }
    /* Creation de la struct class */
    if (IS_ERR(dev_class = class_create(THIS_MODULE, "sud_class"))) {
        pr_err("Impossible de creer la struct class\n");
        goto r_class;
    }

/* Creation du peripherique */
if (IS_ERR(device_create(dev_class, NULL, dev, NULL, "sud_device"))) {
    pr_err("Impossible de creer le peripherique\n");
    goto r_device;
}

//===============================
// Configuration du GPIO de sortie
//===============================
// 1) Verifier si le GPIO est valide ou non
if (gpio_is_valid(GPIO_21_OUT) == false) {
    pr_err("GPIO %d n’est pas valide\n", GPIO_21_OUT);
    goto r_device;
}
// 2) Requete sur le GPIO
if (gpio_request(GPIO_21_OUT, "GPIO_21_OUT") < 0) {
    pr_err("ERREUR: GPIO %d gpio_request\n", GPIO_21_OUT);
    goto r_gpio_out;
}
// 3) Configurer le GPIO en sortie
gpio_direction_output(GPIO_21_OUT, 0);

//===============================
// Configuration du GPIO d’entree
//===============================
// 1) Verifier si le GPIO est valide ou non
if (gpio_is_valid(GPIO_25_IN) == false) {
    pr_err("GPIO %d n’est pas valide\n", GPIO_25_IN);
    goto r_gpio_in;
}
// 2) Requete sur le GPIO
if (gpio_request(GPIO_25_IN, "GPIO_25_IN") < 0) {
    pr_err("ERREUR: GPIO %d gpio_request\n", GPIO_25_IN);
    goto r_gpio_in;
}
// 3) Configurer le GPIO en entree
gpio_direction_input(GPIO_25_IN);
/*
** Le code suivant est desactive sur Raspberry
** (si la macro EN_DEBOUNCE est definie en haut)
*/
#ifndef EN_DEBOUNCE
    // Debouncer le bouton avec un delai de 200ms
    if (gpio_set_debounce(GPIO_25_IN, 200) < 0) {
        pr_err("ERREUR: gpio_set_debounce - %d\n", GPIO_25_IN);
        // goto r_gpio_in;
    }
#endif

// 4) Recuperer le numero d’IRQ pour le GPIO
GPIO_irqNumber = gpio_to_irq(GPIO_25_IN);
pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);

if (request_irq(GPIO_irqNumber,
               (void *)gpio_irq_handler,
               IRQF_TRIGGER_RISING,
               "sud_device",
               NULL)) {
    pr_err("driver_input_output: impossible d’enregistrer l’IRQ ");
    goto r_gpio_in;
}

pr_info("Insertion du pilote de peripherique... OK!!\n");
return 0;

r_gpio_in:
    gpio_free(GPIO_25_IN);
r_gpio_out:
    gpio_free(GPIO_21_OUT);
r_device:
    device_destroy(dev_class, dev);
r_class:
    class_destroy(dev_class);
r_del:
    cdev_del(&sud_cdev);
r_unreg:
    unregister_chrdev_region(dev, 1);
return -1;
}

/* Fonction de finalisation du module */
static void __exit sud_driver_exit(void) {
    free_irq(GPIO_irqNumber, NULL);
    gpio_free(GPIO_25_IN);
    gpio_free(GPIO_21_OUT);
    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&sud_cdev);
    unregister_chrdev_region(dev, 1);
    pr_info("Suppression du pilote de peripherique... OK!!\n");
}

module_init(sud_driver_init);
module_exit(sud_driver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("SUD");
MODULE_DESCRIPTION("Driver avec gestion des interruptions (avec des GPIOs)");
MODULE_VERSION("1.0");


