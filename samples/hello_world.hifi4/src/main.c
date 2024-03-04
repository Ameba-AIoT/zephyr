/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdio.h>
#include <stdint.h>
#include <xtensa/tie/xt_hifi4.h>
#define castxcc(type_,ptr) (type_ *)(ptr)

void test1(void);
void test2(void);

int main(void)
{
    printf("Hello World! hifi4: %s\n", CONFIG_BOARD);
    test1();
    test2();
    return 0;
}

void test1(void)
{
    printf("TEST1:MOV\n");
    printf("GOLDEN_Y:0, 1, 2, 3, 4, 5, 6, 7,\n");
    int x[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    int y[8] = {0};
    ae_int32x2 v_x;
    ae_int32x2 *px, *py;
    px = (ae_int32x2*)x;
    py = (ae_int32x2*)y;
    for (int i = 0; i < 4; i++) {
        AE_L32X2_IP(v_x, px, 8);
        AE_S32X2_IP(v_x, py, 8);
    }

    printf("OUTPUT_Y:");
    for (int i = 0; i < 8; i++) {
        printf("%d, ", y[i]);
    }
    printf("\n");
}

void test2(void)
{
    printf("TEST2:FIR\n");
    printf("GOLDEN_Y:44, 45, 51, -13, -53, -171, -146, 8, 113, 507, 275, 52, 212, -5324, -5829, -1741,\n");
    int i;
    int16_t x[64] = {
        0, 10596, -2341, 21824, 8346, 17113, 25104, 10223,
        30248, 14190, 19136, 23311, 5817, 21731, 3631, 5915,
        7944, -10099, 4213, -13234, -10701, -7842, -23553, -8026,
        -22323, -17318, -11803, -23855, -6198, -17019, -9682, -2151,
        -11694, 6598, -2668, 4843, 12034, 2487, 18948, 8769,
        14176, 18978, 7634, 20889, 8749, 11471, 13735, 838,
        11608, -1451, 0, 1422, -11153, -789, -12679, -10379,
        -7760, -18160, -6505, -15852, -11606, -7037, -14905, -1917
    };

    int16_t h[16] = {
        -38, -46, 64, 96, -143, -209, 296, 409,
        -555, -745, 998, 1349, -1877, -2785, 4823, 14747
    };
    int16_t y[16] = {0};
    ae_int16x4 v_x0, v_x1, v_x2, v_x3, v_h;
    ae_int16x4 *px, *ph, *py;
    px = (ae_int16x4*)x;
    ph = (ae_int16x4*)h;
    py = (ae_int16x4*)y;
    ae_int64 q0, q1, q2, q3;
    ae_valign ay = AE_ZALIGN64();
    ae_int64 z = AE_CVTQ56A32S(0);

    q0 = z; q1 = z; q2 = z; q3 = z;

    for ( i = 0; i < 4; i++) {
        AE_L16X4_IP(v_x0, px, 8);
        AE_L16X4_IP(v_x1, px, 8);
        AE_L16X4_IP(v_x2, px, 8);
        AE_L16X4_IP(v_x3, px, 8);

        AE_L16X4_IP(v_h, ph, 8);

        AE_MULAAAAQ16(q0, v_x0, v_h);
        AE_MULAAAAQ16(q1, v_x1, v_h);
        AE_MULAAAAQ16(q2, v_x2, v_h);
        AE_MULAAAAQ16(q3, v_x3, v_h);

        ae_f32x2 t0 = AE_TRUNCA32X2F64S(q0, q1, 33);
        ae_f32x2 t1 = AE_TRUNCA32X2F64S(q2, q3, 33);

        AE_SA16X4_IP(AE_ROUND16X4F32SASYM(t0, t1), ay, castxcc(ae_int16x4,py));
    }

    printf("OUTPUT_Y:");
    for ( i = 0; i < 16; i++) {
        printf("%d, ", y[i]);
    }
    printf("\n");
}