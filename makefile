all:
	@g++ -Ofast -g -Wall -Wextra -pedantic -std=gnu++17 tests/*.cpp -o tests/test_runner
	@./tests/test_runner
