#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h> // copy_to/from_user()
#include <linux/gpio.h>    // GPIO

#include <linux/interrupt.h>
#include <linux/err.h>

/*
* La gestion de l'instabilité des entrées (debouncing) n'est pas disponible sur Raspberry Pi.
* Ce code est ajouté pour éviter les faux positifs (qui risquent de déclencher plusieurs IRQ pour un seul événement).
* D'autres matériels peuvent supporter le debouncing, ce code est alors inutile.
* Pour éliminer ce code en extra, il suffit de commenter la macro ci-dessous.
* Le code est inspiré de : https://raspberrypi.stackexchange.com/questions/8544/gpio-interrupt-debounce
*
* Pour utiliser le Debounce matériel (s'il existe), il faut commenter la macro EN_DEBOUNCE.
*/
#define EN_DEBOUNCE
#ifdef EN_DEBOUNCE
#include <linux/jiffies.h>
extern unsigned long volatile jiffies; // variable du kernel qui mesure le nombre
                                       // d'interruptions de l'horloge depuis le démarrage du système (donne une "mesure "
                                       // du temps)
unsigned long old_jiffies = 0;
#endif
/* Fin du code extra pour la gestion du Debounce */

// La LED est connectée en sortie à ce GPIO
#define GPIO_21_OUT (21)
// L'interrupteur est connecté en entrée à ce GPIO
#define GPIO_25_IN (25)
// GPIO_25_IN valeur (bascule) à affecter à la LED (0 -> éteinte, 1 -> allumée).
unsigned int led_toggle = 0;
// Utilisé pour stocker le numéro d'IRQ du GPIO
unsigned int GPIO_irqNumber;

/* Fonction de traitement du GPIO 25. Cette fonction est
* appelée à chaque front montant.
*/
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    static unsigned long flags = 0;

#ifdef EN_DEBOUNCE
    unsigned long diff = jiffies - old_jiffies;
    if (diff < 200)
    {
        return IRQ_HANDLED;
    }
    old_jiffies = jiffies;
#endif
	led_toggle = (0x01 ^ led_toggle); // on permute l'ancienne valeur
	gpio_set_value(GPIO_21_OUT, led_toggle); // on positionne le GPIO GPIO_21_OUT
	pr_info("interruption survenue : GPIO_21_OUT : %d\n", gpio_get_value(GPIO_21_OUT));
	local_irq_restore(flags); // on réactive de nouveau les interruptions dans le CPU en cours en restaurant leur état antérieur avant désactivation
	return IRQ_HANDLED; // on retourne que l'interruption a bien été traitée
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

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = sud_read,
    .write = sud_write,
    .open = sud_open,
    .release = sud_release,
};

static int sud_open(struct inode *inode, struct file *file)
{
    pr_info("Fichier périphérique ouvert...!!!\n");
    return 0;
}

static int sud_release(struct inode *inode, struct file *file)
{
    pr_info("Fichier périphérique fermé...!!!\n");
    return 0;
}
static ssize_t sud_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    uint8_t gpio_state = 0;
    // Lecture de la valeur du GPIO
    gpio_state = gpio_get_value(GPIO_21_OUT);
    // Écriture du résultat vers l'espace utilisateur
    len = 1;
    if (copy_to_user(buf, &gpio_state, len) > 0) {
        pr_err("ERREUR: Certains octets n'ont pas pu être copiés vers l'espace utilisateur\n");
    }
    pr_info("sud_read: GPIO_21 = %d\n", gpio_state);
    return 0;
}

static ssize_t sud_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    uint8_t rec_buf[10] = {0};
    if (copy_from_user(rec_buf, buf, len) > 0) {
        pr_err("ERREUR: Certains octets n'ont pas pu être copiés depuis l'espace utilisateur\n");
    }
    pr_info("sud_write : GPIO_21 Set = %c\n", rec_buf[0]);
    if (rec_buf[0] == '1') {
        // On met la valeur du GPIO (en sortie) à HIGH
        gpio_set_value(GPIO_21_OUT, 1);
    } else if (rec_buf[0] == '0') {
        // On met la valeur du GPIO (en sortie) à LOW
        gpio_set_value(GPIO_21_OUT, 0);
    } else {
        pr_err("Commande inconnue : merci de donner soit 1 soit 0\n");
    }
    return len;
}

static int __init sud_driver_init(void)
{
    // Allocation du nombre Majeur
    if (alloc_chrdev_region(&dev, 0, 1, "sud_Dev") < 0)
    {
        pr_err("Impossible d'allouer le nombre Majeur\n");
        goto r_unreg;
    }
    pr_info("Majeur = %d Mineur = %d\n", MAJOR(dev), MINOR(dev));

    // Création de la structure cdev
    cdev_init(&sud_cdev, &fops);

    // Ajout du périphérique caractère au système
    if (cdev_add(&sud_cdev, dev, 1) < 0)
    {
        pr_err("Impossible d'ajouter le périphérique au système\n");
        goto r_del;
    }

    // Création de la structure class
    if (IS_ERR(dev_class = class_create(THIS_MODULE, "sud_class")))
    {
        pr_err("Impossible de créer la structure class\n");
        goto r_class;
    }

    // Création du périphérique
    if (IS_ERR(device_create(dev_class, NULL, dev, NULL, "sud_device")))
    {
        pr_err("Impossible de créer le périphérique\n");
        goto r_device;
    }

    // Configuration du GPIO de sortie
    // Vérifier si le GPIO est valide
    if (gpio_is_valid(GPIO_21_OUT) == false)
    {
        pr_err("GPIO %d n'est pas valide\n", GPIO_21_OUT);
        goto r_device;
    }

    // Requête sur le GPIO
    if (gpio_request(GPIO_21_OUT, "GPIO_21_OUT") < 0)
    {
        pr_err("ERREUR: GPIO %d gpio_request\n", GPIO_21_OUT);
        goto r_gpio_out;
    }
	
	// 3) c o n f i g u r e r l e GPIO en s o r t i e
	gpio_direction_output(GPIO_21_OUT, 0 );
	
	// 1) Vérifier si le GPIO est valide ou non
	if (gpio_is_valid(GPIO_25_IN) == false) {
		pr_err("GPIO %d n’est pas valide\n", GPIO_25_IN);
		goto r_gpio_in;
	}

	// 2) Requête sur le GPIO
	if (gpio_request(GPIO_25_IN, "GPIO_25_IN") < 0) {
		pr_err("ERREUR: GPIO %d gpio_request\n", GPIO_25_IN);
		goto r_gpio_in;
	}

	// 3) Configurer le GPIO en entrée
	gpio_direction_input(GPIO_25_IN);

/*
** Le code suivant est désactivé sur Raspberry
** (si la macro EN_DEBOUNCE est définie en haut)
*/
	#ifndef EN_DEBOUNCE
	// Debouncer le bouton avec un délai de 200ms
	if (gpio_set_debounce(GPIO_25_IN, 200) < 0) {
		pr_err("ERREUR: gpio_set_debounce − %d\n", GPIO_25_IN);
		// goto r_gpio_in;
	}
	#endif

	// 4) Récupérer le numéro d’IRQ pour le GPIO
	GPIO_irqNumber = gpio_to_irq(GPIO_25_IN);
	pr_info("GPIO_irqNumber = %d\n", GPIO_irqNumber);

	if (request_irq(GPIO_irqNumber,    // numéro d’IRQ (obtenu par gpio_to_irq)
                (void *)gpio_irq_handler, // Fonction de traitement de l’IRQ
                IRQF_TRIGGER_RISING,       // Le traitement sera déclenché sur le front montant
                "sud_device",              // Utilisé pour identifier le nom du périphérique qui utilise cet IRQ
                NULL)) {                   // ID du périphérique pour les IRQs partagés
		pr_err("driver_input_output: impossible d’enregistrer l’IRQ\n");
		goto r_gpio_in;
	}

	pr_info("Insertion du pilote de périphérique...OK!!!\n");
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

// Fonction de finalisation du module
static void __exit sud_driver_exit(void)
{
    free_irq(GPIO_irqNumber, NULL); // on libère l’IRQ à la fin
    gpio_free(GPIO_25_IN);          // on libère le GPIO d’entrée
    gpio_free(GPIO_21_OUT);         // on libère le GPIO de sortie
    device_destroy(dev_class, dev); // on détruit le périphérique (pour supprimer /dev/...)
    class_destroy(dev_class);       // on détruit la classe (pour supprimer /dev/sud_device)
    cdev_del(&sud_cdev);            // on supprime /dev/sud_device
    unregister_chrdev_region(dev, 1); // on libère le majeur et ses mineurs
    pr_info("Suppression du pilote de périphérique...OK!!!\n");
}

module_init(sud_driver_init);
module_exit(sud_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SUD");
MODULE_DESCRIPTION("Driver avec gestion des interruptions (avec des GPIOs)");
MODULE_VERSION("1.0");
