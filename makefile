all:
	@g++ -g -Wall -Wextra -std=gnu++17 tests/*.cpp -o tests/test_runner
	@./tests/test_runner
