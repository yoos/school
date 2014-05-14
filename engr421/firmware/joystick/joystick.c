/* Simple joystick controller to read joystick inputs and relay them to an
 * output device.
 *
 * Output Packet Structure:
 *
 * struct command_pkt {
 *   uint8_t header[3];
 *   uint8_t roll;
 *   uint8_t pitch;
 *   uint8_t yaw;
 *   uint8_t throttle;
 *   uint8_t trailer[3];
 * }
 */

#include <fcntl.h>
#include <linux/joystick.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

// Axis 0: Roll
// Axis 1: Pitch
// Axis 3: Yaw
// Axis 2 and 4: Throttle

static bool armed = false;
static int8_t throttle = 0;
static int8_t axes[3];
static uint8_t button;

void *read_thread(void *param) {
  int *fd = (int *) param;

  struct js_event e;
  while(true) {
    read(*fd, &e, sizeof(e));

    if((e.type) == JS_EVENT_AXIS) {
      // Scale the 16-bit value to a signed 8-bit value
      uint8_t scaled = (uint8_t) ((e.value + 32767) / (256*2));

      // TODO: Pull axis configuration out

      // Check the axis
      if(e.number == 0 || e.number == 1 || e.number == 3) {
        if(e.number == 1) {
          scaled = -scaled;
        }

        // Ignore the real axix 2 (throttle 1) and make it yaw
        int n = e.number == 3 ? 2 : e.number;
        axes[n] = scaled;
      } else if(e.number == 4) {
        throttle = (-scaled) / 2;

        if(!armed && throttle < 10) {
          armed = true;
          printf("Arming!\n");
        }
      }

      printf("Roll: %3d Pitch: %3d Yaw: %3d Throttle: %3d\n", axes[0], axes[1], axes[2], throttle);
    }
	else if (e.type == JS_EVENT_BUTTON) {
		printf("Button %d: %d\n", e.number, e.value);
		if (e.number == 0) {
			button = e.value;
		}
	}
  }
}

void *write_thread(void *param) {
  int *outfd = (int *) param;

  while(true) {
    // Don't send any commands unless the controller has been reset to a low
    // throttle.
    if(armed) {
      printf("Sending!\n");
      // Build and write the packet
      uint8_t buf[1] = {(button << 7) + (0x7f & axes[0])};

      ssize_t ret = write(*outfd, buf, sizeof(buf));
      if(ret == -1) {
        perror("Failed to write to output");
      }
    } else {
      // printf("Not armed!\n");
    }

    nanosleep((struct timespec[]){{0, 1000000000 * 0.020}}, NULL); // 50Hz
  }
}

int main(int argc, char **argv) {
  if(argc < 3) {
    fprintf(stderr, "Usage: %s [jspath] [outpath]\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  char *jspath = argv[1];
  int fd = open(jspath, O_RDONLY);
  if(fd == -1) {
    perror("Error opening joystick device");
    exit(EXIT_FAILURE);
  }

  char *outpath = argv[2];
  int outfd = open(outpath, O_WRONLY);
  if(outfd == -1) {
    perror("Error opening output device");
    exit(EXIT_FAILURE);
  }

  pthread_t reader_thread;
  pthread_t writer_thread;

  pthread_create(&reader_thread, NULL, read_thread, &fd);
  pthread_create(&writer_thread, NULL, write_thread, &outfd);

  pthread_join(reader_thread, NULL);
  pthread_join(writer_thread, NULL);

  return EXIT_SUCCESS;
}
