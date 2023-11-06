#include <thread>
#include <iostream>
#include <random>

void printme()
{
	std::cout << "hello" << std::endl;
	
	std::random_device rd;     // Only used once to initialise (seed) engine
	std::mt19937 rng(rd());    // Random-number engine used (Mersenne-Twister in this case)
	std::uniform_real_distribution<> dis(0.0f,10.0f); // Guaranteed unbiased
	float random_integer = dis(rng);

	std::cout << random_integer << std::endl;
	
	return;
}


int main() {
	std::thread t1 = std::thread(printme);
	t1.join();
	return 0;
}