#include "i2c.h"
#include "my_i2c.h"

#define EEPROM_ADDRESS	0xA0
#define EEPROM_PAGESIZE	64
#define EEPROM_TIMEOUT	1000
#define I2C_TEST_MAX 70
uint8_t my_I2C_r_buf[I2C_TEST_MAX]= {0};
uint8_t my_I2C_w_buf[I2C_TEST_MAX]= {1,2,3,4};
void my_i2c_test(void)
{
    int i=0;
    for(i=0; i<I2C_TEST_MAX; i++)
    {
        my_I2C_w_buf[i]=i+1;
    }

    HAL_I2C_Mem_Read(&hi2c1,0X0A0,0X0000,I2C_MEMADD_SIZE_16BIT,my_I2C_r_buf,I2C_TEST_MAX,1000);
    printf("I2C write:%d\n",my_I2C_w_buf[0]);
    HAL_I2C_Mem_Write(&hi2c1,0X0A0,0X0000,I2C_MEMADD_SIZE_16BIT,my_I2C_w_buf,I2C_TEST_MAX,1000);
//HAL_Delay(5);
    HAL_I2C_Mem_Read(&hi2c1,0X0A0,0X0000,I2C_MEMADD_SIZE_16BIT,my_I2C_r_buf,I2C_TEST_MAX,1000);
    for(i=0; i<I2C_TEST_MAX; i++)
    {
        printf("%3d-",my_I2C_r_buf[i]);
    }
}
