/****************************************************************************
 * jitest/jitest_main.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <semaphore.h>
#include <pthread.h>

#include <nuttx/semaphore.h>
#include <nuttx/timers/timer.h>

#if defined( CONFIG_ARCH_CHIP_STM32)
#include "stm32gpio.h"
#define GPIOCONFIG stm32_configgpio
#define GPIOWRITE  stm32_gpiowrite
#elif defined( CONFIG_ARCH_CHIP_STM32L4)
#include "stm32l4gpio.h"
#define GPIOCONFIG stm32l4_configgpio
#define GPIOWRITE  stm32l4_gpiowrite
#else
#error this app is only for stm32
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


//config at  10 Hz = 100ms = 100000us
//config at 100 Hz = 10ms  = 10000us

#define CONFIG_JITEST_DEVNAME "/dev/timer0"

#define CONFIG_JITEST_INTERVAL 10000 //timer interval

#define CONFIG_JITEST_DELAY 100000 //sleep interval
#define CONFIG_JITEST_NSAMPLES 500 //sleep count

#define GPIO_IRQ (GPIO_PORTC|GPIO_PIN11|GPIO_OUTPUT|GPIO_SPEED_50MHz|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR)
#define GPIO_TSK (GPIO_PORTD|GPIO_PIN2 |GPIO_OUTPUT|GPIO_SPEED_50MHz|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int global_irq;
int global_tsk;
int quit;

sem_t trigger;

/****************************************************************************
 * timer_handler
 ****************************************************************************/

static bool timer_handler(FAR uint32_t *next_interval_us)
{
  /* This handler may:
   *
   * (1) Modify the timeout value to change the frequency dynamically, or
   * (2) Return false to stop the timer.
   */
  global_irq = !global_irq;
  GPIOWRITE(GPIO_IRQ, global_irq); 
  sem_post(&trigger);
  return true;
}

static void *thread(void *arg)
  {
    while(!quit)
      {
        sem_wait(&trigger);
        global_tsk = !global_tsk;
        GPIOWRITE(GPIO_TSK, global_tsk); 
      }
    return NULL;
  }

static void *hog(void *arg)
  {
    int i;
    while(!quit)
      {
        i++;
        printf("%d",i);
      }
    return NULL;
  }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * timer_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int jitest_main(int argc, char *argv[])
#endif
{
  struct timer_sethandler_s handler;
  int ret;
  int fd;
  int i;
  pthread_t id,idhog;

  global_irq = 0;
  GPIOCONFIG(GPIO_IRQ);

  global_tsk = 0;
  GPIOCONFIG(GPIO_TSK);

  quit = 0;
  sem_init(&trigger,0,0);
  sem_setprotocol(&trigger, SEM_PRIO_NONE); //flip-flop fifo, avoids prio inh if enabled

  pthread_create(&id, NULL, thread, NULL);
  pthread_setschedprio(id, PTHREAD_DEFAULT_PRIORITY);

  if(argc>1)
    {
      pthread_create(&idhog, NULL, hog, NULL);
      pthread_setschedprio(idhog, PTHREAD_DEFAULT_PRIORITY);

      if(!strcmp(argv[1], "hi"))
        {
          pthread_setschedprio(id, PTHREAD_DEFAULT_PRIORITY+10);
        }
    }

  /* Open the timer device */

  printf("Open %s\n", CONFIG_JITEST_DEVNAME);

  fd = open(CONFIG_JITEST_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "ERROR: Failed to open %s: %d\n",
              CONFIG_JITEST_DEVNAME, errno);
      return EXIT_FAILURE;
    }


  /* Set the timer interval */

  printf("Set timer interval to %lu\n",
         (unsigned long)CONFIG_JITEST_INTERVAL);

  ret = ioctl(fd, TCIOC_SETTIMEOUT, CONFIG_JITEST_INTERVAL);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set the timer interval: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Attach the timer handler
   *
   * NOTE: If no handler is attached, the timer stop at the first interrupt.
   */

  printf("Attach timer handler\n");

  handler.newhandler = timer_handler;
  handler.oldhandler = NULL;

  ret = ioctl(fd, TCIOC_SETHANDLER, (unsigned long)((uintptr_t)&handler));
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to set the timer handler: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Start the timer */

  printf("Start the timer\n");

  ret = ioctl(fd, TCIOC_START, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to start the timer: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  for (i = 0; i < CONFIG_JITEST_NSAMPLES; i++)
    {
      usleep(CONFIG_JITEST_DELAY);
    }

  /* Stop the timer */

  printf("Stop the timer\n");

  ret = ioctl(fd, TCIOC_STOP, 0);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to stop the timer: %d\n", errno);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Close the timer driver */

  printf("Finished\n");
  close(fd);
  return EXIT_SUCCESS;
}
