#include <I2C_F103.h>
#include <Delay_F103.h>
#include <MLX90621.h>

void ExtractPTATParameters(uint8_t *eeData, paramsMLX90621 *mlx90621);
void ExtractTgcParameters(uint8_t *eeData, paramsMLX90621 *mlx90621);
void ExtractKsTaParameters(uint8_t *eeData, paramsMLX90621 *mlx90621);
void ExtractKsToParameters(uint8_t *eeData, paramsMLX90621 *mlx90621);
void ExtractAlphaParameters(uint8_t *eeData, paramsMLX90621 *mlx90621);
void ExtractOffsetParameters(uint8_t *eeData, paramsMLX90621 *mlx90621);


int MLX90621_DumpEE(uint8_t *eeData)
{
     return MLX90621_I2CReadEEPROM(0x50, 0, 256, eeData);
}


int MLX90621_GetFrameData(uint16_t *frameData)
{
    int error = 1;
        
    error = MLX90621_I2CRead(0x60, 0x02, 0, 1, 66, frameData);
       
    return error;    
}

int MLX90621_Configure(uint8_t *eeData)
{
    int error = 1;
    uint16_t value;
    
    error = MLX90621_I2CWrite(0x60, 0x04, 0xAA, eeData[247]);
    
    if (error != 0)
    {
        return error;
    }
    
    value = 256*eeData[246] + eeData[245];      
    value = value | 0x0400;
    error = MLX90621_I2CWrite(0x60, 0x03, 0x55, value);
    
    return error;      
    
}   

int MLX90621_GetOscillatorTrim(uint16_t *oscTrim)
{
    int error = 1;
        
    error = MLX90621_I2CRead(0x60, 0x02, 0x93, 0, 1, oscTrim); 
       
    return error;    
}    

int MLX90621_GetConfiguration(uint16_t *cfgReg)
{
    int error = 1;
        
    error = MLX90621_I2CRead(0x60, 0x02, 0x92, 0, 1, cfgReg); 
       
    return error;    
}   
 

int MLX90621_ExtractParameters(uint8_t *eeData, paramsMLX90621 *mlx90621)
{
    int error = 0;
    
    ExtractPTATParameters(eeData, mlx90621);
    ExtractTgcParameters(eeData, mlx90621);
    ExtractKsTaParameters(eeData, mlx90621);
    ExtractKsToParameters(eeData, mlx90621);
    ExtractAlphaParameters(eeData, mlx90621);
    ExtractOffsetParameters(eeData, mlx90621);
        
    return error;

}

//------------------------------------------------------------------------------

int MLX90621_SetResolution(uint8_t resolution)
{
    uint16_t cfgReg;
    int value;
    int error;
    
    value = (resolution & 0x03) << 4;
    
    error = MLX90621_GetConfiguration(&cfgReg);
    
    if(error == 0)
    {
        value = (cfgReg & 0xFFCF) | value;
        error = MLX90621_I2CWrite(0x60, 0x03, 0x55, value);        
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90621_GetCurResolution()
{
    uint16_t cfgReg;
    int resolution;
    int error;
    
    error = MLX90621_GetConfiguration(&cfgReg);
    if(error != 0)
    {
        return error;
    }    
    resolution = (cfgReg & 0x0030) >> 4;
    
    return resolution; 
}

//------------------------------------------------------------------------------

int MLX90621_SetRefreshRate(uint8_t refreshRate)
{
    uint16_t cfgReg;
    int value;
    int error;
    
    value = refreshRate & 0x0F;
    
    error = MLX90621_GetConfiguration(&cfgReg);
        
    if(error == 0)
    {
        value = (cfgReg & 0xFFF0) | value;
        error = MLX90621_I2CWrite(0x60, 0x03, 0x55, value);        
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90621_GetRefreshRate()
{
    uint16_t cfgReg;
    int rr;
    int error;
    
    error = MLX90621_GetConfiguration(&cfgReg);
    if(error != 0)
    {
        return error;
    }    
    rr = (cfgReg & 0x000F);
    
    return rr; 
}

//------------------------------------------------------------------------------

void MLX90621_CalculateTo(uint16_t *frameData, const paramsMLX90621 *params, float emissivity, float tr, float *result)
{
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float irDataCP;
    float irData;
    float alphaCompensated;
    float Sx;
    float To;
    
    ta = MLX90621_GetTa(frameData, params);
    
    ta4 = (ta + 273.15);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = (tr + 273.15);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    taTr = tr4 - (tr4-ta4)/emissivity;
    
//------------------------- To calculation -------------------------------------    
        
    irDataCP = frameData[65];  

    if(irDataCP > 32767)
    {
        irDataCP = irDataCP - 65536;
    }

    irDataCP = irDataCP - (params->cpA + params->cpB * (ta - 25));
    
    for( int pixelNumber = 0; pixelNumber < 64; pixelNumber++)
    {    
        irData = frameData[pixelNumber];
        if(irData > 32767)
        {
            irData = irData - 65536;
        }
        
        irData = irData - (params->ai[pixelNumber] + params->bi[pixelNumber] * (ta - 25));
        
        irData = irData - params->tgc * irDataCP;
        irData = irData / emissivity;
        
        alphaCompensated = params->alpha[pixelNumber] - params->tgc * params->cpAlpha;
        alphaCompensated = alphaCompensated *(1 + params->KsTa * (ta - 25));
                    
        Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
        Sx = sqrt(sqrt(Sx)) * params->ksTo;            
        
        To = sqrt(sqrt(irData/(alphaCompensated * (1 - params->ksTo * 273.15) + Sx) + taTr)) - 273.15;                     
                                    
        result[pixelNumber] = To;
    
    }
}

//------------------------------------------------------------------------------

void MLX90621_GetImage(uint16_t *frameData, const paramsMLX90621 *params, float *result)
{
    float ta;
    float irDataCP;
    float irData;
    float alphaCompensated;
    
    ta = MLX90621_GetTa(frameData, params);
    
//------------------------- Image calculation -------------------------------------    
        
    irDataCP = frameData[65];  

    if(irDataCP > 32767)
    {
        irDataCP = irDataCP - 65536;
    }

    irDataCP = irDataCP - (params->cpA + params->cpB * (ta - 25));
    
    for( int pixelNumber = 0; pixelNumber < 64; pixelNumber++)
    {    
        irData = frameData[pixelNumber];
        if(irData > 32767)
        {
            irData = irData - 65536;
        }
        
        irData = irData - (params->ai[pixelNumber] + params->bi[pixelNumber] * (ta - 25));
        
        irData = irData - params->tgc * irDataCP;
        
        alphaCompensated = params->alpha[pixelNumber] - params->tgc * irDataCP;
        alphaCompensated = alphaCompensated *(1 + params->KsTa * (ta - 25));
                                                        
        result[pixelNumber] = irData/alphaCompensated;                     
    
    }
}

//------------------------------------------------------------------------------

float MLX90621_GetTa(uint16_t *frameData, const paramsMLX90621 *params)
{
    float ptat;
    float ta;
    
    ptat = frameData[64];
    if(ptat > 32767)
    {
        ptat = ptat - 65536;
    }
       
    ta = params->vTh25 - ptat;
    ta = 4*params->kT2*ta;
    ta = params->kT1*params->kT1 - ta;
    ta = sqrt(ta)-params->kT1;
    ta = ta / (2*params->kT2);
    ta = ta + 25;
    
    return ta;
}

//------------------------------------------------------------------------------

void ExtractPTATParameters(uint8_t *eeData, paramsMLX90621 *mlx90621)
{
    float kt1;
    float kt2;
    int16_t vth25;
    int kt1Scale = 0;
    int kt2Scale = 0;
    int resolution = 3;
    uint16_t data;
    
    resolution = resolution - MLX90621_GetCurResolution();
    kt1Scale = (eeData[210] & 0xF0) >> 4;
    kt2Scale = eeData[210] & 0x0F;
    kt2Scale = kt2Scale + 10;
    kt1Scale = kt1Scale + resolution;
    kt2Scale = kt2Scale + resolution;
    
    data = (eeData[219]<<8) + eeData[218];
    vth25 = data;
    vth25 = vth25/(1<<resolution);
	
    kt1 = (eeData[221]<<8) + eeData[220];
    if (kt1 > 32767)
    {
        kt1 = kt1 - 65536;
    }  
    
    kt1 = kt1 / (1<<kt1Scale);  
    
    kt2 = (eeData[223]<<8) + eeData[222];
    if (kt2 > 32767)
    {
        kt2 = kt2 - 65536;
    }  
    
    kt2 = kt2 / (1<<kt2Scale); 
    
    mlx90621->vTh25 = vth25;
    mlx90621->kT1 = kt1;    
    mlx90621->kT2 = kt2;
      
}

//------------------------------------------------------------------------------

void ExtractTgcParameters(uint8_t *eeData, paramsMLX90621 *mlx90621)
{
    float tgc;
    tgc = eeData[216]/32.0f;
        
    mlx90621->tgc = tgc;        
}

//------------------------------------------------------------------------------

void ExtractKsTaParameters(uint8_t *eeData, paramsMLX90621 *mlx90621)
{
    float KsTa;
    KsTa = (eeData[231] << 8) + eeData[230];
    if(KsTa > 32767)
    {
        KsTa = KsTa - 65536;
    }
    KsTa = KsTa / (1<<20);
    
    mlx90621->KsTa = KsTa;
}

//------------------------------------------------------------------------------

void ExtractKsToParameters(uint8_t *eeData, paramsMLX90621 *mlx90621)
{
    int scale;
        
    scale = eeData[192] & 0x0F;
    scale = scale + 8;
    
    mlx90621->ksTo = eeData[196];
       
    if(mlx90621->ksTo > 127)
    {
        mlx90621->ksTo = mlx90621->ksTo - 256;
    }
    mlx90621->ksTo = mlx90621->ksTo / (1 << scale);
    
}

//------------------------------------------------------------------------------

void ExtractAlphaParameters(uint8_t *eeData, paramsMLX90621 *mlx90621)
{    
    float alphaScale;
    float deltaScale;
    uint8_t resScale;
    float alphaCom; 
    float alphaTemp;
    float temp;
    
    alphaScale = eeData[226];
    deltaScale = eeData[227];
    resScale = 3 - MLX90621_GetCurResolution();
    alphaCom = (eeData[225]<<8) + eeData[224];
    alphaScale = pow(2,alphaScale);
    deltaScale = pow(2,deltaScale);
    alphaCom = alphaCom / alphaScale;
  
    for(int i = 0; i < 64; i++)
    {
        temp = eeData[128+i];
        temp = temp / deltaScale;        
        alphaTemp = (alphaCom + temp)/(1<<resScale);
        mlx90621->alpha[i] = alphaTemp;
    } 
    
    alphaTemp = (eeData[215]<<8) + eeData[214];
    alphaTemp = alphaTemp / alphaScale;
    
    mlx90621->cpAlpha = alphaTemp / (1<<resScale);
}

//------------------------------------------------------------------------------

void ExtractOffsetParameters(uint8_t *eeData, paramsMLX90621 *mlx90621)
{
    uint8_t aScale;
    float bScale;
    uint8_t resScale;
    int16_t aCom;
    float aTemp;
    float bTemp;
    uint16_t data;
    
    aScale = eeData[217]>>4;
    bScale = eeData[217] & 0x0F;
    resScale = 3 - MLX90621_GetCurResolution();
    bScale = pow(2, (float)(bScale+resScale));
    
    data = (eeData[209]<<8) + eeData[208];
    aCom = data;    

    for(int i=0; i<64; i++)
    {
        aTemp = eeData[i]<<aScale; 
        bTemp = eeData[64+i];
        if (bTemp > 127)
        {
            bTemp = bTemp - 256;
        }            
        bTemp = bTemp / bScale;
        
        mlx90621->ai[i] = (aCom + aTemp) / (1<<resScale);
        mlx90621->bi[i] = bTemp;
    } 
    
    aTemp = (eeData[212]<<8) + eeData[211];
    if (aTemp > 32767)
    {
        aTemp = aTemp - 65536;
    }    
    aTemp = aTemp / (1<<resScale); 
    
    bTemp = eeData[213];
    if (bTemp > 127)
    {
        bTemp = bTemp - 256;
    }     
    bTemp = bTemp / bScale;      
    
    mlx90621->cpA = aTemp;
    mlx90621->cpB = bTemp;
}

void MLX90621_I2CInit()
{   
	I2C_Config();
	I2C_Stop();
}

int MLX90621_I2CReadEEPROM(uint8_t slaveAddr, uint8_t startAddress, int nMemAddressRead, uint8_t *data)
{
    uint8_t sa;                                
    int cnt = 0;
    uint8_t cmd = 0;
    uint8_t i2cData[256] = {0};
    uint8_t *p;
    
    p = data;
    sa = (slaveAddr << 1);
    cmd = startAddress;
    
    I2C_Stop();
    Delay_us(5);    
		I2C_Start();
		I2C_Address(sa);
		I2C_Write(cmd);
		I2C_Stop();
		
    sa = sa | 0x01;
    
		I2C_Start();
		I2C_Read(sa,i2cData,nMemAddressRead);
    I2C_Stop();   
    
    for(cnt=0; cnt < nMemAddressRead; cnt++)
    {
        *p++ = i2cData[cnt];
    }
    
    return 0;   
} 

int MLX90621_I2CRead(uint8_t slaveAddr,uint8_t command, uint8_t startAddress, uint8_t addressStep, int nMemAddressRead, uint16_t *data)
{
    uint8_t sa;                               
    int cnt = 0;
    int i = 0;
    uint8_t cmd[4] = {0,0,0,0};
    uint8_t i2cData[132] = {0};
    uint16_t *p;
    
    p = data;
    sa = (slaveAddr << 1);
    cmd[0] = command;
    cmd[1] = startAddress;
    cmd[2] = addressStep;
    cmd[3] = nMemAddressRead;
    
    I2C_Stop();
    Delay_us(5);
		I2C_Start();
		I2C_Address(sa);
		I2C_WriteBuf(cmd,4);
		
    sa = sa | 0x01;
    I2C_Start();
		I2C_Read(sa,i2cData,2*nMemAddressRead);
		I2C_Stop();   
    
    for(cnt=0; cnt < nMemAddressRead; cnt++)
    {
        i = cnt << 1;
        *p++ = (uint16_t)i2cData[i+1]*256 + (uint16_t)i2cData[i];
    }
    
    return 0;   
} 

int MLX90621_I2CWrite(uint8_t slaveAddr, uint8_t command, uint8_t checkValue, uint16_t data)
{
    uint8_t sa;
    uint8_t cmd[5] = {0,0,0,0,0};
    static uint16_t dataCheck;
    

    sa = (slaveAddr << 1);
    cmd[0] = command;
    cmd[2] = data & 0x00FF;
    cmd[1] = cmd[2] - checkValue;
    cmd[4] = data >> 8;
    cmd[3] = cmd[4] - checkValue;

    I2C_Stop();
		Delay_us(5);
		I2C_Start();
		Delay_us(5);
		I2C_Address(sa);
		I2C_WriteBuf(cmd,5);
		I2C_Stop();   
    
    MLX90621_I2CRead(slaveAddr, 0x02, 0x8F+command, 0, 1, &dataCheck);
    
    if ( dataCheck != data)
    {
        return -2;
    }    
    
    return 0;
}