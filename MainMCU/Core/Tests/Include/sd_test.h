#ifndef SD_TEST_H
#define SD_TEST_H

#define SD_TASK_READ_COMPLETE_BIT (1U << 0)
#define SD_TASK_WRITE_COMPLETE_BIT (1U << 1)

void sd_test_task(void *args);

#endif