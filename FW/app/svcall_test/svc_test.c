/* SPDX-License-Identifier: MIT */
/*
 * svc_test.c - SVC_Handler demo utility
 * 
 * Copyright (c) 2025 Michael Kaa
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

typedef struct {
    uint32_t svc_num;   // Извлеченный номер SVC 
    uint32_t args[4];   // Аргументы
    uint32_t svc_ret;   // return value
} svc_debug_t;

volatile svc_debug_t svc_debug_info = {0};

void print_svc_debug(void);

// Номер svc должен быть константой известной на этапе компиляции
#define SVC_NUM (85)

// Макрос для вызова SVC с номером вызова
#define SVC_CALL(num, arg0, arg1, arg2, arg3) \
    __asm volatile(                           \
        "mov r0, %0\n"                        \
        "mov r1, %1\n"                        \
        "mov r2, %2\n"                        \
        "mov r3, %3\n"                        \
        "svc %4\n"                            \
        :                                     \
        : "r" (arg0), "r" (arg1), "r" (arg2), "r" (arg3), "I" (num) \
        : "r0", "r1", "r2", "r3", "memory"    \
    )
    
#ifdef BAREMETAL
int ucmd_tscv(int argc, char* argv[])
#define ENDL "\r\n"
#else
int main(int argc, char* argv[])
#define ENDL "\n"
#endif // BAREMETAL

{
    uint32_t args[4] = {0};

    if (argc < 2) {
        printf("Usage: scv [arg0] [arg1] [arg2] [arg3]" ENDL);
        printf("  [argN] - decimal args" ENDL);
        return -EINVAL;
    }
    
    // Парсим аргументы
    for (int i = 0; i < (argc - 1); i++) {
        if (sscanf(argv[i+1], "%lu", &args[i]) != 1) {
            printf("EINVAL %d: %s" ENDL, i, argv[i+1]);
            return -EINVAL;
        }
    }
    
    /* Вызываем SVC с переданными аргументами */
    printf("super visor call: num=%u, args=[%lu, %lu, %lu, %lu]" ENDL, SVC_NUM, args[0], args[1], args[2], args[3]);

    // svc_num пока не используем никак - хардкодим 85, оно должно быть константой времени компиляции
    SVC_CALL(SVC_NUM, args[0], args[1], args[2], args[3]);
    __asm volatile(
        "ldr r1, =svc_debug_info\n"
        "str r0, [r1, #20]\n"  // svc ret value
    );
    print_svc_debug(); 

    return 0;
}

void print_svc_debug(void)
{
    printf("svc %lu (0x%02lX)" ENDL, svc_debug_info.svc_num, svc_debug_info.svc_num);
    printf("arg[0]: %lu" ENDL, svc_debug_info.args[0]);
    printf("arg[1]: %lu" ENDL, svc_debug_info.args[1]);
    printf("arg[2]: %lu" ENDL, svc_debug_info.args[2]);
    printf("arg[3]: %lu" ENDL, svc_debug_info.args[3]);

    printf("svc call return : %lu" ENDL, svc_debug_info.svc_ret);
}


// +------------+
// |   xPSR     |  ← [SP + 28] 
// +------------+
// |   PC       |  ← [SP + 24] (адрес после SVC)
// +------------+
// |   LR       |  ← [SP + 20] (EXC_RETURN)
// +------------+
// |   R12      |  ← [SP + 16] 
// +------------+
// |   R3       |  ← [SP + 12] (arg3)
// +------------+
// |   R2       |  ← [SP + 8]  (arg2)
// +------------+
// |   R1       |  ← [SP + 4]  (arg1)
// +------------+
// |   R0       |  ← [SP + 0]  (arg0/результат)
// +------------+


__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile(
        // MSP or PSP?
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r1, msp\n"    // MSP
        "mrsne r1, psp\n"    // PSP

        "push {r1, lr}\n"

        // Получаем PC из стека
        "ldr r2, [r1, #24]\n"

        // Получаем адрес инструкции SVC
        "subs r2, #2\n"         // PC указывает на следующую инструкцию после SVC
        
        // Читаем инструкцию SVC (16 бит в Thumb)
        "ldrh r0, [r2]\n"
        
        // Извлекаем номер SVC 
        "and r0, #0xff\n"
        
        // <---
        "bl svc_proc\n"

        "pop {r1, lr}\n"
        
        "str r0, [r1, #0]\n"
        
        "bx lr\n"
    );
}

uint32_t svc_proc(uint32_t svc, uint32_t* arg) {
    svc_debug_info.svc_num = svc;
    svc_debug_info.args[0] = arg[0];
    svc_debug_info.args[1] = arg[1];
    svc_debug_info.args[2] = arg[2];
    svc_debug_info.args[3] = arg[3];

    return svc;
}


// макет кода проверяющего были ли использованы регистры FPU и сохраняющий их
// скорее всего нерабочий
// __attribute__((naked)) void SVC_Handler(void)
// {
//     __asm volatile(
//         "tst lr, #4                      \n"
//         "ite eq                          \n"
//         "mrseq r0, msp                   \n"
//         "mrsne r0, psp                   \n"
        
//         // Сохраняем регистры (базовый набор для всех Cortex-M)
//         "push {r4-r11, lr}              \n"
        
//     #if defined(CORTEX_M4F) || defined(CORTEX_M7)
//         // Проверяем, использовался ли FPU
//         "tst lr, #0x10                   \n"
//         "bne 1f                          \n"
//         // Сохраняем FPU регистры
//         #ifdef CORTEX_M7
//             // Для M7 с double precision
//             "vstmdb r0!, {d0-d15}        \n"
//         #else
//             // Для M4 с single precision
//             "vstmdb r0!, {s0-s31}        \n"
//         #endif
//     "1:                                 \n"
//     #endif
        
//         // Общая часть обработки
//         "mov r4, r0                      \n" // Сохраняем SP
        
//         // ... ваша логика обработки SVC ...
        
//     #if defined(CORTEX_M4F) || defined(CORTEX_M7)
//         // Восстанавливаем FPU если нужно
//         "tst lr, #0x10                   \n"
//         "bne 2f                          \n"
//         #ifdef CORTEX_M7
//             "vldmia r4!, {d0-d15}        \n"
//         #else
//             "vldmia r4!, {s0-s31}        \n"
//         #endif
//     "2:                                 \n"
//     #endif
        
//         "pop {r4-r11, pc}                \n"
//     );
// }


#undef ENDL