#ifndef SD_TEST_H
#define SD_TEST_H

#define SD_TASK_READ_COMPLETE_BIT (1U << 0)
#define SD_TASK_WRITE_COMPLETE_BIT (1U << 1)
#define SD_TASK_OPEN_COMPLETE_BIT (1U << 2)
#define SD_TASK_CLOSE_COMPLETE_BIT (1U << 3)

void sd_test_task(void *args);

#endif