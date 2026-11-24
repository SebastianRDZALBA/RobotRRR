/*
 * Componente cinematica: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "cinematica.h"

bool cd_calcular_mm(double base_oper_deg, double hombro_oper_deg,
                           double codo_nom_deg,
                           double *px_mm, double *py_mm, double *pz_mm)
{
    if (!px_mm || !py_mm || !pz_mm ||
        !isfinite(base_oper_deg) || !isfinite(hombro_oper_deg) ||
        !isfinite(codo_nom_deg)) return false;
    const double q1 = CD_SIGNO_Q1 * base_oper_deg * CD_RAD_POR_GRADO;
    const double q2 = CD_SIGNO_Q2 * hombro_oper_deg * CD_RAD_POR_GRADO;
    const double q3 = CD_SIGNO_Q3 *
             (codo_nom_deg - CD_CODO_READY_NOMINAL_GRADOS) * CD_RAD_POR_GRADO;
    const double radial = CD_L2_MM*sin(q2) + CD_L3_MM*sin(q2+q3);
    const double lateral = -(CD_DELTA2_MM + CD_DELTA3_MM);
    *px_mm = cos(q1)*radial - sin(q1)*lateral;
    *py_mm = sin(q1)*radial + cos(q1)*lateral;
    *pz_mm = CD_L1_MM + CD_L2_MM*cos(q2) + CD_L3_MM*cos(q2+q3);
    return isfinite(*px_mm) && isfinite(*py_mm) && isfinite(*pz_mm);
}

double ci_normalizar_rad(double angulo)
{
    return atan2(sin(angulo),cos(angulo));
}

bool ci_limites_generales(double base_deg, double hombro_deg,
                                double codo_deg)
{
    const double t=CI_TOL_ANGULO_GRADOS;
    const double h_min=(HOMBRO_LIMITE_MIN_PULSOS-HOMBRO_READY_PULSOS) *
                       360.0/PULSOS_VUELTA_HOMBRO;
    const double h_max=(HOMBRO_LIMITE_MAX_PULSOS-HOMBRO_READY_PULSOS) *
                       360.0/PULSOS_VUELTA_HOMBRO;
    return isfinite(base_deg) && isfinite(hombro_deg) && isfinite(codo_deg) &&
           base_deg >= BASE_LIMITE_IZQUIERDA_ESTIMADO-t &&
           base_deg <= BASE_LIMITE_DERECHA_ESTIMADO+t &&
           hombro_deg >= h_min-t && hombro_deg <= h_max+t &&
           codo_deg >= 0.0-t && codo_deg <= 180.0+t;
}

int ci_calcular_mm(double px, double py, double pz,
                          ci_solucion_t soluciones[CI_MAX_SOLUCIONES])
{
    if (!soluciones || !isfinite(px) || !isfinite(py) || !isfinite(pz))
        return 0;
    const double d=-(CD_DELTA2_MM+CD_DELTA3_MM);
    const double h=pz-CD_L1_MM;
    const double radio_xy=hypot(px,py);
    if (!isfinite(radio_xy) || radio_xy < fabs(d)-CI_TOL_CARTESIANA_MM)
        return 0;
    const double r2=radio_xy*radio_xy-d*d;
    const double modulo_r=sqrt(fmax(0.0,r2));
    const double cos_q3=(modulo_r*modulo_r+h*h-CD_L2_MM*CD_L2_MM-
                         CD_L3_MM*CD_L3_MM)/(2.0*CD_L2_MM*CD_L3_MM);
    // Comparar en mm, no un epsilon fijo del coseno dependiente de escala.
    const double distancia_plano=hypot(modulo_r,h);
    if (!isfinite(cos_q3) ||
        distancia_plano > CD_L2_MM+CD_L3_MM+CI_TOL_CARTESIANA_MM ||
        distancia_plano < fabs(CD_L2_MM-CD_L3_MM)-CI_TOL_CARTESIANA_MM)
        return 0;
    const double q3_abs=acos(fmax(-1.0,fmin(1.0,cos_q3)));
    const double direccion_base=atan2(py,px);
    int total=0;
    for (int ir=0;ir<2;ir++) {
        const double r=(ir==0 ? modulo_r : -modulo_r);
        const double q1=ci_normalizar_rad(direccion_base-atan2(d,r));
        for (int ie=0;ie<2;ie++) {
            const double q3=(ie==0 ? q3_abs : -q3_abs);
            const double q2=ci_normalizar_rad(
                atan2(r,h)-atan2(CD_L3_MM*sin(q3),
                                  CD_L2_MM+CD_L3_MM*cos(q3)));
            const double q1g=q1*CI_GRADOS_POR_RAD;
            const double q2g=q2*CI_GRADOS_POR_RAD;
            const double q3g=q3*CI_GRADOS_POR_RAD;
            const double b=q1g/CD_SIGNO_Q1;
            const double hombro=q2g/CD_SIGNO_Q2;
            const double c=CD_CODO_READY_NOMINAL_GRADOS+q3g/CD_SIGNO_Q3;
            if (!isfinite(b)||!isfinite(hombro)||!isfinite(c)) continue;
            bool repetida=false;
            for (int j=0;j<total;j++) {
                if (fabs(soluciones[j].q1_deg-q1g)<CI_TOL_SOLUCION_REPETIDA &&
                    fabs(soluciones[j].q2_deg-q2g)<CI_TOL_SOLUCION_REPETIDA &&
                    fabs(soluciones[j].q3_deg-q3g)<CI_TOL_SOLUCION_REPETIDA) {
                    repetida=true;
                    break;
                }
            }
            if (repetida) continue;
            double rx,ry,rz;
            if (!cd_calcular_mm(b,hombro,c,&rx,&ry,&rz)) continue;
            const double error=hypot(hypot(px-rx,py-ry),pz-rz);
            if (!isfinite(error) || error>CI_TOL_CARTESIANA_MM+1e-8) continue;
            const bool teorica_exacta=ci_limites_generales(b,hombro,c);
            const double hmin=(HOMBRO_LIMITE_MIN_PULSOS-HOMBRO_READY_PULSOS)*
                                360.0/PULSOS_VUELTA_HOMBRO;
            const double hmax=(HOMBRO_LIMITE_MAX_PULSOS-HOMBRO_READY_PULSOS)*
                                360.0/PULSOS_VUELTA_HOMBRO;
            const bool dentro_sin_tolerancia=
                b>=BASE_LIMITE_IZQUIERDA_ESTIMADO &&
                b<=BASE_LIMITE_DERECHA_ESTIMADO &&
                hombro>=hmin && hombro<=hmax && c>=0.0 && c<=180.0;
            ci_solucion_t *sol=&soluciones[total++];
            *sol=(ci_solucion_t){
                .q1_deg=q1g,.q2_deg=q2g,.q3_deg=q3g,
                .base_oper_deg=b,.hombro_oper_deg=hombro,
                .codo_nominal_deg=c,.error_reconstruccion_mm=error,
                .limites_articulares_generales=teorica_exacta,
                .usa_tolerancia_geometrica=error>1e-6,
                .usa_tolerancia_articular=teorica_exacta && !dentro_sin_tolerancia
            };
            if (total >= CI_MAX_SOLUCIONES) return total;
        }
    }
    return total;
}

void mostrar_cinematica_directa(void)
{
    if (paro) return;
    printf("\n===== CINEMATICA DIRECTA V6 (SOLO TELEMETRIA) =====\n");
    if (paro || sistema_cerrado || !base_home || !hombro_home ||
        !servo_codo.pwm_activo) {
        printf("P_x, P_y, P_z: NO CALCULABLES con referencia confiable.\n");
        if (!base_home) printf("  Falta HOME de BASE.\n");
        if (!hombro_home) printf("  Falta HOME de HOMBRO.\n");
        if (!servo_codo.pwm_activo) printf("  CODO sin PWM: angulo fisico desconocido.\n");
        if (paro || sistema_cerrado) printf("  Sistema detenido o cerrado.\n");
        return;
    }
    const double b = (double)base_pulsos * 360.0/PULSOS_VUELTA_BASE;
    const double h = (double)(hombro_rel_pulsos-HOMBRO_READY_PULSOS) *
                     360.0/PULSOS_VUELTA_HOMBRO;
    const double c = codo_grados_de_pwm(servo_codo.us_actual);
    double px=0.0, py=0.0, pz=0.0;
    if (!cd_calcular_mm(b,h,c,&px,&py,&pz)) {
        printf("ERROR: entradas o resultado de cinematica no validos.\n");
        return;
    }
    printf("Modelo: L1=%.1f, L2=%.1f, L3=%.1f, D2=%.1f, D3=%.1f mm.\n",
           CD_L1_MM,CD_L2_MM,CD_L3_MM,CD_DELTA2_MM,CD_DELTA3_MM);
    printf("Referencias: q1=%+.3f, q2=%+.3f, q3=%+.3f grados (signos validados).\n",
           CD_SIGNO_Q1*b, CD_SIGNO_Q2*h,
           CD_SIGNO_Q3*(c-CD_CODO_READY_NOMINAL_GRADOS));
    printf("P_x=%+.3f mm | P_y=%+.3f mm | P_z=%+.3f mm\n",px,py,pz);
    printf("Estimacion: pasos NEMA y PWM del codo, SIN encoders.\n");
    printf("Signos q1=-base, q2=+hombro, q3=90-codo verificados experimentalmente.\n");
}

