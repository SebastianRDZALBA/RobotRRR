#ifndef BRAZO_MOD_HOME_READY_H
#define BRAZO_MOD_HOME_READY_H
#include "estado_robot.h"

bool codo_postura_segura_prehome(void);

bool codo_postura_segura_para_home_hombro(void);

bool codo_postura_segura_para_retorno(void);

bool despeje_hombro_5_si_switch(void);

bool postura_segura_prehome(void);

bool pasar_a_preparado(void);

bool robot_a_preparado(void);

bool robot_a_home(void);

void preguntar_preparado_despues_home(void);

bool primer_home(void);

#endif
