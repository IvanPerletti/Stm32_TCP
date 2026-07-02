
#ifndef TANALOGPORT_H
#define TANALOGPORT_H


#define MV_COUNT_CONV_FACTOR	(1221)	/* Vref=2500mV; for AEC: Vin/Vadc=2 => ((2500/4095)*2) = (0.6105*2) = 1.221 mV/count */
#define MAX_AEC_mV_VALUE		(5000)	/* 5V */


// Shared Definitions --------------------------------------------------------
typedef enum {
	ANALOG_ABC_ERR = 0,
	ANALOG_CM7 = 1,
	ANALOG_CM10 = 2,
	ANALOG_AEC1 = 3,
	ANALOG_AEC2 = 4,
	ANALOG_3VE = 5,
	ANALOG_5V = 6,
	ANALOG_NUMEL
} enumAnalogPort; // please modify TAnalogPort.read( )too
// Shared macro
#define AEC_TO_mV(val):	((unsigned long)(val) * 3300) >> 12 // same as val*3300mV/4095
#define IS_ANALOG_PORT(X)	(	((short)X >= 0)	&&	(X < ANALOG_NUMEL)	)

class TAnalogPort {
private:
	int kk;
	volatile unsigned short  u16aAdcValue[ANALOG_NUMEL]; /*! array with ADC values*/
	bool bIsOpen;
public:
	TAnalogPort(void);
	~TAnalogPort(void);
	unsigned short read ( enumAnalogPort portNum );
	void read ( unsigned short * pu16Arr );
	void deInit(void);
	void open(void);
	void close(void);
	bool isOpen(void);
private:
	void init(void);
	void configDMA(void);
	void configGpioForADC(void);
	void configADC(void);
};

extern TAnalogPort tAnalogPort;
#endif /*!< TANALOGPORT_H */
