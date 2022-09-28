
# BQ76920

This library is made for the STM32 HAL (Hardware Abstraction Library) platform. This is a Battery Monitoring AFE library from TEXAS for use with STM32XXX microcontroller.



[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)

## Usage/Examples

```javascript
/* USER CODE BEGIN Includes */
#include "BQ76920.h"
/* USER CODE BEGIN PV */
BQ76920_t BMS;
/* USER CODE END PV */

int main(void)
{
    MX_I2C2_Init();
    BMS.i2cHandle = &hi2c2;
    BMS.bootPort = GPIOC;
    BMS.bootPin  = GPIO_PIN_9
    
    /* USER INIT*/
    BQ76920_Initialise(&BMS,&hi2c2);
    BQ76920_Enable_ADC(&BMS);

    /* while loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        float Vcell[4], Vpack, Ipack, PackTemp;
        Vpack = getPackVoltage(&BMS);
        for(int i =0;i<=3;i++)
        {
            if( i == 3)
            {
                Vcell[i] = getCellVoltage(&BMS,VC4);
            }
            else
                Vcell[i] = getCellVoltage(&BMS,VC1+(2*i));
            HAL_Delay(15); 
        }
        Ipack = getCurrent(&BMS);
        PackTemp = getDieTemp(&BMS);
        
        
        EnableBalanceCell(&BMS);
        HAL_Delay(200);
        DisableBalanceCell(&BMS);

        BQ76920_Shutdown(&BMS);
    }
    /* USER CODE END 3 */
}
```


    
## Features
- Compatible with 4S Litium Battery 
- Get Cell Voltage
- Get Battery pack Voltage
- Get Battery pack Current
- Get Die Temperature
- Enable/Disable Balancing mode
- Shutdown



## Authors

- [@NAWAT.L](https://www.github.com/LLL2542)


## License

[MIT](https://choosealicense.com/licenses/mit/)
