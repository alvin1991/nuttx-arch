
/* IOCTL commands */

#define ACDC_IOC_POWER_ON 			_PWRIOC(100)        /* set power on */
#define ACDC_IOC_POWER_OFF 			_PWRIOC(101)        /* set power off */
#define ACDC_IOC_RESET	 			_PWRIOC(102)        /* set device reset */
#define ACDC_IOC_SET_VOL 			_PWRIOC(103)        /* set output voltage */
#define ACDC_IOC_READ_VIN			_PWRIOC(105)        /* read input voltage */
#define ACDC_IOC_READ_VOUT			_PWRIOC(106)        /* read output voltage */
#define ACDC_IOC_READ_IOUT			_PWRIOC(107)        /* read output current */
#define ACDC_IOC_READ_POUT			_PWRIOC(108)        /* read output power */
#define ACDC_IOC_READ_PIN			_PWRIOC(109)        /* read input power */
#define ACDC_IOC_READ_TEMP			_PWRIOC(110)        /* read device temperature */

/*acdc power device priv struct*/
struct acdc_power_dev_s
{
  /* Bus management */

  struct i2c_master_s *i2c;      /* I2C device  */
  uint32_t             freq;     /* I2C bus speed */
  uint8_t              i2c_addr; /* 7-bit unshifted I2C device address */

  /* Driver management */

  sem_t                sem;      /* file write access serialization */
  uint8_t              refs;     /* Nr of times the device has been opened */
  bool                 readonly; /* Flags */

  /* priv data */
  struct power_acdc_status_s *status;
  FAR const void *priv;

};

/*acdc power device status*/
struct power_acdc_status_s
{
  float vol_in;
  float vol_out;
  float power_in;
  float power_out;
  float I_out;			//output current
  float temperature;
};

int acdc_power_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,uint8_t const acdc_i2c_addr);
