# Rocket PID Attitude Controller

## Files needed

```
main/rocket_pid.h
main/rocket_pid.c
main/test_pid.c
scripts/generate_tests.py
scripts/tests/verify_all.sh
scripts/tests/create_test.sh
scripts/tests/run_test.sh
```

## Build and run all 50 tests

```bash
gcc main/rocket_pid.c main/test_pid.c -o main/test_pid -lm
python3 scripts/generate_tests.py
./scripts/tests/verify_all.sh
```

## Create a custom test

```bash
./scripts/tests/create_test.sh my_test
```

Edit `all_tests/my_test.txt`, then:

```bash
./scripts/tests/run_test.sh my_test --save
./scripts/tests/verify_all.sh
```
