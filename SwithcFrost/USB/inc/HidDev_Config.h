
#ifndef HIDDEV_CONFIG_H_INCLUDED
#define HIDDEV_CONFIG_H_INCLUDED
#include <stdint.h>
#define USB_VID           0x3210
#define USB_PID           0x0098  // SWITHC FROSTER 	
#define USB_NUM           0x0098 // Серийный номер устройства.

#define ENDP_IN_SIZE      8  //4 Размер в байтах пакета конечной точки типа Input (0 - 64). Направление из МК в ПК.
#define ENDP_OUT_SIZE     8//1 Размер в байтах пакета конечной точки типа Output (0 - 64). Направление из ПК в МК.
#define ENDP_FEATURE_SIZE 0  // Размер в байтах пакета типа Feature (0 - 64). Двунаправленный обмен через нулевую конечную точку.

#define ENDP_IN_INTERVAL  64 // Интервал в миллисекундах опроса конечной точки типа Input (1 - 255).
#define ENDP_OUT_INTERVAL 64 // Интервал в миллисекундах опроса конечной точки типа Output (1 - 255).


// ---------------------- Служебные данные. Не изменять! ----------------------------

#define HID_REPORT_SIZE ( ((((ENDP_IN_SIZE)>0) && ((ENDP_IN_SIZE)<=64)) ? 12 : 0) + \
                         ((((ENDP_OUT_SIZE)>0) && ((ENDP_OUT_SIZE)<=64)) ? 12 : 0) + \
                         ((((ENDP_FEATURE_SIZE)>0) && ((ENDP_FEATURE_SIZE)<=64)) ? 12 : 0) + 8 )

void HID_SendBuff(uint8_t *Buff, uint8_t Size);

#endif /* HIDDEV_CONFIG_H_INCLUDED */
