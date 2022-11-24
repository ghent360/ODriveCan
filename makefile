all:
	@g++ -Ofast -g -Wall -Wextra -pedantic -std=gnu++17 TaskManagerTest.cpp -o test_runner
	@./test_runner
