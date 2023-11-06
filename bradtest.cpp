#include <thread>
#include <iostream>

void printme()
{
	std::cout << "hello" << std::endl;
}


int main() {
	std::thread t1 = std::thread(printme);
}