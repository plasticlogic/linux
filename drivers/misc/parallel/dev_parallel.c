/**
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * Based upon the character device example from Derec Molloy
 */

#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <asm/uaccess.h>          // Required for the copy to user function
#define  DEVICE_NAME "parallel"    ///< The device will appear at /dev/parallel using this value
#define  CLASS_NAME  "ebb"        ///< The device class -- this is a character device driver

MODULE_LICENSE("GPL");            ///< The license type -- this affects available functionality
MODULE_AUTHOR("Robert Pohlink");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("Parallel Driver for the epson S1D13524 EPDC");  ///< The description -- see modinfo
MODULE_VERSION("0.1");            ///< A version number to inform users

#define RUDDOCK 1
//---------------------------------------------------------------------------
//
// Local Definitions
//
//---------------------------------------------------------------------------
#define GPIO0_RESET_L		26  //< GPIO0_26
#define GPIO1_HDC			17  //< GPIO1_17
#define GPIO0_HRD_L			4   //< GPIO0_4
#define GPIO0_HWE_L			2   //< GPIO0_2
#define GPIO0_HCS_L			5   //< GPIO0_5
#define GPIO3_HRDY			19  //< GPIO3_19
#define GPIO3_HIRQ			21  //< GPIO3_21
//Define GPIO0 parameters 
#define GPIO0_START_ADDR	0x44E07000
#define GPIO0_END_ADDR		0x44E08FFF
#define GPIO0_MAP_SIZE (GPIO0_END_ADDR - GPIO0_START_ADDR)

//Define GPIO1 parameters 
#define GPIO1_START_ADDR	0x4804C000
#define GPIO1_END_ADDR		0x4804DFFF
#define GPIO1_MAP_SIZE (GPIO1_END_ADDR - GPIO1_START_ADDR)

//Define GPIO2 parameters 
#define GPIO2_START_ADDR	0x481AC000
#define GPIO2_END_ADDR		0x481ADFFF
#define GPIO2_MAP_SIZE (GPIO2_END_ADDR - GPIO2_START_ADDR)

//Define GPIO3 parameters 
#define GPIO3_START_ADDR	0x481AE000
#define GPIO3_END_ADDR		0x481AFFFF
#define GPIO3_MAP_SIZE (GPIO3_END_ADDR - GPIO3_START_ADDR)

#define GPIO_OE				0x134
#define GPIO_DATAIN			0x138
#define GPIO_DATAOUT		0x13C
#define GPIO_SETDATAOUT		0x194
#define GPIO_CLEARDATAOUT	0x190


//Define GPIO Clock parameters
#define CM_PER_BASE					0x44e00000
#define CM_MAP_SIZE					0x1000
#define CM_WKUP_BASE				0x44e00400
#define CM_WKUP_SIZE				0x0100
#define CM_GPIO0_CLK_CONFIG_OFFSET	0x408
#define CM_GPIO1_CLK_CONFIG_OFFSET	0xAC
#define CM_GPIO2_CLK_CONFIG_OFFSET	0xB0
#define CM_GPIO3_CLK_CONFIG_OFFSET	0xB4

#define BBB_OE_HWR			0xFC3E0001

#define BS_HRDY_TIMEOUT 10000000
#define S1D13524_EPDC_MAX_HRDY_TIMEOUT 10000
//---------------------------------------------------------------------------
//
// Local Globals
//
//---------------------------------------------------------------------------
void * _map_CM;
void * _map_GPIO0;
void * _map_GPIO1;
void * _map_GPIO2;
void * _map_GPIO3;

int _hdb_dir;

volatile void *pGpio0_clk_config = NULL;
volatile void *pGpio1_clk_config = NULL;
volatile void *pGpio2_clk_config = NULL;
volatile void *pGpio3_clk_config = NULL;

volatile unsigned int *gpio0_oe_addr = NULL;
volatile unsigned int *gpio0_datain_addr = NULL;
volatile unsigned int *gpio0_dataout_addr = NULL;
volatile unsigned int *gpio0_setdataout_addr = NULL;
volatile unsigned int *gpio0_cleardataout_addr = NULL;


volatile unsigned int *gpio1_oe_addr = NULL;
volatile unsigned int *gpio1_datain_addr = NULL;
volatile unsigned int *gpio1_dataout_addr = NULL;
volatile unsigned int *gpio1_setdataout_addr = NULL;
volatile unsigned int *gpio1_cleardataout_addr = NULL;

volatile unsigned int *gpio2_oe_addr = NULL;
volatile unsigned int *gpio2_datain_addr = NULL;
volatile unsigned int *gpio2_dataout_addr = NULL;
volatile unsigned int *gpio2_setdataout_addr = NULL;
volatile unsigned int *gpio2_cleardataout_addr = NULL;


volatile unsigned int *gpio3_oe_addr = NULL;
volatile unsigned int *gpio3_datain_addr = NULL;
volatile unsigned int *gpio3_dataout_addr = NULL;
volatile unsigned int *gpio3_setdataout_addr = NULL;
volatile unsigned int *gpio3_cleardataout_addr = NULL;

volatile short message[1228800] = {0,};

//---------------------------------------------------------------------------
//
// Local Function Prototypes
//
//---------------------------------------------------------------------------
void init_gpio( void );

void open_gpio( void );
void close_gpio( void );
void gpio_hdb_dir( int v );
void set_gpio0_dir( int pin, int val );
int  get_gpio0_dir( int pin );
int  get_gpio0_val( int pin );
void set_gpio0_val( int pin, int val );

void set_gpio1_dir( int pin, int val );
int  get_gpio1_dir( int pin );
int  get_gpio1_val( int pin );
void set_gpio1_val( int pin, int val );

void set_gpio2_dir( int pin, int val );
int  get_gpio2_dir( int pin );
int  get_gpio2_val( int pin );
void set_gpio2_val( int pin, int val );

void set_gpio3_dir( int pin, int val );
int  get_gpio3_dir( int pin );
int  get_gpio3_val( int pin );
void set_gpio3_val( int pin, int val );

unsigned int  gpio_hdb_read( void );
void gpio_hdb_write( unsigned int val );
void gpio_hdc( int v ) { set_gpio1_val( GPIO1_HDC, v ); }
void gpio_hrd_l( int v ) { set_gpio0_val( GPIO0_HRD_L, v ); }
void gpio_hwe_l( int v ) { set_gpio0_val( GPIO0_HWE_L, v ); }
void gpio_hcs_l( int v ) { set_gpio0_val( GPIO0_HCS_L, v ); }
int  gpio_hrdy( void ) { return get_gpio3_val( GPIO3_HRDY ); }

int wait_for_ready( void );
static int    majorNumber;                  ///< Stores the device number -- determined automatically

static int    numberOpens = 0;              ///< Counts the number of times the device is opened
static struct class*  parallelClass  = NULL; ///< The device-driver class struct pointer
static struct device* parallelDevice = NULL; ///< The device-driver device struct pointer

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *filep, unsigned short *buffer, size_t len,  loff_t *offset);
static ssize_t dev_write(struct file *filep, const short *buffer, size_t len, loff_t *offset);

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
};



void open_gpio( void )
{
	_map_CM = ioremap_nocache(CM_PER_BASE, CM_MAP_SIZE);
	if (_map_CM == NULL)
		printk("Unable to remap GPIO Clock memory!\n");

	_map_GPIO0 = ioremap_nocache(GPIO0_START_ADDR, GPIO0_MAP_SIZE);
	if (_map_GPIO0 == NULL)
		printk("Unable to remap GPIO0!\n");
		
	_map_GPIO1 = ioremap_nocache(GPIO1_START_ADDR, GPIO1_MAP_SIZE);
	if (_map_GPIO1 == NULL)
		printk("Unable to remap GPIO1!\n");

	_map_GPIO2 = ioremap_nocache(GPIO2_START_ADDR, GPIO2_MAP_SIZE);
	if (_map_GPIO2 == NULL)
		printk("Unable to remap GPIO2!\n");
		
	_map_GPIO3 = ioremap_nocache(GPIO3_START_ADDR, GPIO3_MAP_SIZE);
	if (_map_GPIO3 == NULL)
		printk("Unable to remap GPIO3!\n");
}

void _parallel_exit( void )
{
	iounmap( _map_CM );
	iounmap( _map_GPIO0 );
	iounmap( _map_GPIO1 );
	iounmap( _map_GPIO2 );
	iounmap( _map_GPIO3 );
}


void _parallel_init( void )
{
	int reg;
	open_gpio();
	pGpio0_clk_config = (volatile unsigned int *) _map_CM + (CM_GPIO0_CLK_CONFIG_OFFSET/4);
	*((volatile unsigned int *)pGpio0_clk_config) = 2; 
	pGpio2_clk_config = (volatile unsigned int *) _map_CM + (CM_GPIO2_CLK_CONFIG_OFFSET/4);
	*((volatile unsigned int *)pGpio2_clk_config) = 2; 
	pGpio3_clk_config = (volatile unsigned int *) _map_CM + (CM_GPIO3_CLK_CONFIG_OFFSET/4); 
	*((volatile unsigned int *)pGpio3_clk_config) = 2; 
	
	gpio0_oe_addr = (volatile unsigned int *) _map_GPIO0 +  (GPIO_OE/4);
	gpio0_datain_addr = (volatile unsigned int *) _map_GPIO0 +  (GPIO_DATAIN/4);
	gpio0_dataout_addr = (volatile unsigned int *) _map_GPIO0 +  (GPIO_DATAOUT/4);
	gpio0_setdataout_addr = (volatile unsigned int *) _map_GPIO0 +  (GPIO_SETDATAOUT/4);
	gpio0_cleardataout_addr = (volatile unsigned int *) _map_GPIO0 +  (GPIO_CLEARDATAOUT/4);
	gpio2_oe_addr = (volatile unsigned int *) _map_GPIO2 +  (GPIO_OE/4);
	gpio2_datain_addr = (volatile unsigned int *) _map_GPIO2 +  (GPIO_DATAIN/4);
	gpio2_dataout_addr = (volatile unsigned int *) _map_GPIO2 +  (GPIO_DATAOUT/4);
	gpio2_setdataout_addr = (volatile unsigned int *) _map_GPIO2 +  (GPIO_SETDATAOUT/4);
	gpio2_cleardataout_addr = (volatile unsigned int *) _map_GPIO2 +  (GPIO_CLEARDATAOUT/4);
	
	gpio3_oe_addr = (volatile unsigned int *) _map_GPIO3 +  (GPIO_OE/4);
	gpio3_datain_addr = (volatile unsigned int *) _map_GPIO3 +  (GPIO_DATAIN/4);
	gpio3_dataout_addr = (volatile unsigned int *) _map_GPIO3 +  (GPIO_DATAOUT/4);
	gpio3_setdataout_addr = (volatile unsigned int *) _map_GPIO3 +  (GPIO_SETDATAOUT/4);
	gpio3_cleardataout_addr = (volatile unsigned int *) _map_GPIO3 +  (GPIO_CLEARDATAOUT/4);
    //SET DATAOUT REGISTER
    	
	//GPIO0 : Bit5, Bit4, Bit2
    reg = *gpio0_dataout_addr;
    reg = reg | 0x00000034;
    *gpio0_dataout_addr = reg;
	//GPIO0_8 to GPIO0_11,  Bit5, Bit4, Bit2
    reg = *gpio0_oe_addr;
    reg = reg & 0xFFFFF0CB;
    *gpio0_oe_addr = reg;	
	//GPIO2_6 to GPIO2_17
    reg = *gpio2_oe_addr;
    reg = reg & 0xFFFC003F;
    *gpio2_oe_addr = reg;
    _hdb_dir = 0;	

}

void set_gpio0_val(int pin, int val)
{
	unsigned int p = 1 << pin;
	if (val & 0x1)
		*gpio0_setdataout_addr = p;
	else
		*gpio0_cleardataout_addr= p; 
}


int get_gpio0_val(int pin)
{
	unsigned int v = *gpio0_datain_addr;
	return (v >> pin) & 0x1;
}


void set_gpio0_dir( int pin, int val )
{
	unsigned int p = 1 << pin;
	if (val & 0x1) {
		*gpio0_oe_addr |= p;
	}
	else {
		*gpio0_oe_addr &= (~p);
	}
}


int get_gpio0_dir( int pin )
{
	unsigned int v = *gpio0_oe_addr;
	return (v >> pin) & 0x1;
}


void set_gpio1_val(int pin, int val)
{	
	unsigned int p = 1 << pin;
	if (val & 0x1)
		*gpio1_setdataout_addr = p;
	else
		*gpio1_cleardataout_addr = p; 
}


int get_gpio1_val(int pin)
{
	unsigned int v = *gpio1_datain_addr;
	return (v >> pin) & 0x1;
}


void set_gpio1_dir( int pin, int val )
{
	unsigned int p = 1 << pin;
	if (val & 0x1) {
		*gpio1_oe_addr |= p;
	}
	else {
		*gpio1_oe_addr &= (~p);
	}
}


int get_gpio1_dir( int pin )
{
	unsigned int v = *gpio1_oe_addr;
	return (v >> pin) & 0x1;
}


void set_gpio2_val(int pin, int val)
{
	unsigned int p = 1 << pin;
	if (val & 0x1)
		*gpio2_setdataout_addr = p;
	else
		*gpio2_cleardataout_addr= p; 
}


int get_gpio2_val(int pin)
{
	unsigned int v = *gpio2_datain_addr;
	return (v >> pin) & 0x1;
}


void set_gpio2_dir( int pin, int val )
{
	unsigned int p = 1 << pin;
	if (val & 0x1) {
		*gpio2_oe_addr |= p;
	}
	else {
		*gpio2_oe_addr &= (~p);
	}
}



int get_gpio2_dir( int pin )
{
	unsigned int v = *gpio2_oe_addr;
	return (v >> pin) & 0x1;
}


void set_gpio3_val(int pin, int val)
{
	unsigned int p = 1 << pin;
	if (val & 0x1)
		*gpio3_setdataout_addr = p;
	else
		*gpio3_cleardataout_addr= p; 
}


int get_gpio3_val(int pin)
{
	unsigned int v = *gpio3_datain_addr;
	return (v >> pin) & 0x1;
}


void set_gpio3_dir( int pin, int val )
{
	unsigned int p = 1 << pin;
	if (val & 0x1) {
		*gpio3_oe_addr |= p;
	}
	else {
		*gpio3_oe_addr &= (~p);
	}
}


int get_gpio3_dir( int pin )
{
	unsigned int v = *gpio3_oe_addr;
	return (v >> pin) & 0x1;
}


unsigned int gpio_hdb_read(void)
{
	#if RUDDOCK
	unsigned int B15_B12;
	unsigned int B11_B0;
	unsigned int d;
	gpio_hrd_l( 0 );
	wait_for_ready();
	if (!_hdb_dir) 
		gpio_hdb_dir(1);
		
	B15_B12 = *gpio0_datain_addr;
	B11_B0 = *gpio2_datain_addr;
	d = ((B15_B12 & 0x00000F00) << 4) | ((B11_B0 & 0x0003FFC0) >> 6);
	#else
	//FALCON
	unsigned int B15_B12;
	unsigned int B11_B0;
	unsigned int B10;
	unsigned int B11;
    unsigned int B12;
	unsigned int B13;       
	unsigned int d;
	gpio_hrd_l( 0 );
	wait_for_ready();
	if (!_hdb_dir) 
		gpio_hdb_dir(1);
		
	B15_B12 = *gpio0_datain_addr;
	B11_B0 = *gpio2_datain_addr;
	
	
	d = ((B15_B12 & 0x00000F00) << 4) | ((B11_B0 & 0x0003FFC0) >> 6);
	B10 = d & 0x00000400;
	B11 = d & 0x00000800;
	B12 = d & 0x00001000;
	B13 = d & 0x00002000;

	d = d & (0x0000C3FF);
	d = d | (B10 << 1) | (B11 << 2) | (B12 >> 2) | (B13 >> 1);
	#endif
	gpio_hrd_l( 1 );
	//printk(KERN_INFO "%s(): read 0x%04X\n", __func__, d);
	return d;
}

//recommended method of writing to gpios
void gpio_hdb_write(unsigned int val)
{
	//printk(KERN_INFO "%s(): write 0x%04X\n", __func__, val);
	unsigned int B15_B12;
	unsigned int B11_B0;
	#if RUDDOCK
	if (_hdb_dir) 
		gpio_hdb_dir(0);

	gpio_hwe_l( 0 );

	B15_B12 = ((val & 0x0000F000) >>4);
	B11_B0 = ((val & 0x00000FFF) << 6);
	B15_B12 = (*gpio0_datain_addr & 0xFFFFF0FF) | B15_B12;
	B11_B0 = (*gpio2_datain_addr & 0xFFFC003F) | B11_B0;
	*gpio0_dataout_addr = B15_B12;
	*gpio2_dataout_addr = B11_B0;

	gpio_hwe_l( 1 );
	#else
	// FALCON
	unsigned int B10;
	unsigned int B11;
    unsigned int B12;
	unsigned int B13;
	if (_hdb_dir) 
		gpio_hdb_dir(0);

	gpio_hwe_l( 0 );

	
	B10 = val & 0x00000400;
	B11 = val & 0x00000800;
	B12 = val & 0x00001000;
	B13 = val & 0x00002000;	
	
	val &= 0x0000C3FF;
	val = val | (B10 << 2) | (B11 >> 1) | (B12 << 1) | (B13 >> 2);
	
	B15_B12 = ((val & 0x0000F000) >>4);
	B11_B0 = ((val & 0x00000FFF) << 6);       

	B15_B12 = (*gpio0_datain_addr & 0xFFFFF0FF) | B15_B12;
	B11_B0 = (*gpio2_datain_addr & 0xFFFC003F) | B11_B0;
	*gpio0_dataout_addr = B15_B12;
	*gpio2_dataout_addr = B11_B0;

	gpio_hwe_l( 1 );
	#endif	
}


void gpio_hdb_dir(int val)
{
	if (val & 0x1) {
      *gpio0_oe_addr |= 0x00000F00;
	  *gpio2_oe_addr |= 0x0003FFC0;
      _hdb_dir = 1;
   }
   else {
      *gpio0_oe_addr &= 0xFFFFF0FF;
      *gpio2_oe_addr &= 0xFFFC003F;
      _hdb_dir = 0;
   }
}

int wait_for_ready( void )
{
	int d = gpio_hrdy( );
	unsigned long count = 0;

	while ( d == 0 && count < BS_HRDY_TIMEOUT){
		d = gpio_hrdy( );
		count++;
	}
	if (count == BS_HRDY_TIMEOUT)
		printk (KERN_ALERT "HRDY timeout Occurred\n");
		
	return d;
}

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point.
 *  @return returns 0 if successful
 */
static int __init parallel_init(void){
	printk(KERN_ALERT "parallel: Initializing the parallel LKM\n");

   // Try to dynamically allocate a major number for the device -- more difficult but worth it
   majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
   if (majorNumber<0){
      printk(KERN_ALERT "parallel failed to register a major number\n");
      return majorNumber;
   }
   // Register the device class
   parallelClass = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(parallelClass)){                // Check for error and clean up if there is
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(parallelClass);          // Correct way to return an error on a pointer
   }
   // Register the device driver
   parallelDevice = device_create(parallelClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(parallelDevice)){               // Clean up if there is an error
      class_destroy(parallelClass);           // Repeated code but the alternative is goto statements
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to create the device\n");
      return PTR_ERR(parallelDevice);
   }
   _parallel_init();
   return 0;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit parallel_exit(void){
	_parallel_exit();
   device_destroy(parallelClass, MKDEV(majorNumber, 0));     // remove the device
   class_unregister(parallelClass);                          // unregister the device class
   class_destroy(parallelClass);                             // remove the device class
   unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
   //printk(KERN_ALERT "parallel: Goodbye from the LKM!\n");
}

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
	numberOpens++;
   return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, unsigned short *buffer, size_t len, loff_t *offset){
	int error_count = 0;
   short result = gpio_hdb_read()  & (0xFFFF);
   error_count = copy_to_user(buffer, &result, sizeof(result));
   if (error_count!=0){            // if true then have success
      printk(KERN_ALERT "parallel: Failed to send %d characters to the user\n", error_count);
      return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
   }
   //printk(KERN_ALERT "parallel: Sent 0x%04X,(%d characters) to the user\n", result, len);
	
   return 0;
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const short *buffer, size_t len, loff_t *offset){
	if(len>1228800) len = 1228800;
	int i, r;
	
	r = copy_from_user(message, buffer, len*(sizeof(short)));
	
	if(r)
		return -22;
	for(i=0; i<len; i++)
		gpio_hdb_write(message[i] & (0xFFFF));
	return len;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
	return 0;
}

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(parallel_init);
module_exit(parallel_exit);
