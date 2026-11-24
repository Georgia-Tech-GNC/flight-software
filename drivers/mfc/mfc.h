#ifndef MFC_H
#define MFC_H

#define MFC_ADC_MAX_COUNTS 65535.0f //16-bit ADC??
#define MFC_BAT_R_TOP      82000.0f //R20
#define MFC_BAT_R_BOT      19100.0f //R25
#define MFC_SHUNT_OHMS     0.008f   //R33
#define MFC_CUR_GAIN       200.0f  

//battery voltage
float mfc_read_battery_volts(void);
//current
float mfc_read_current_amps(void);
//digital
bool mfc_pyro_global_enable(void);
bool mfc_pyro1_status(void);
bool mfc_pyro2_status(void);

//print readings to UART
typedef struct {
    float battery_volts;
    float current_amps;
    bool  pyro_global_en;
    bool  pyro1_stat;
    bool  pyro2_stat;
} mfc_metrics_t;

//return readings and pyro status
mfc_metrics_t mfc_get_metrics(void);
//turn on pyro channels
void mfc_pyro1_set(bool on);
void mfc_pyro2_set(bool on);

#endif 
