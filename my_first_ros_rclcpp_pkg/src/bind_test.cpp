#include <iostream>
#include <functional>

using namespace std;

int test(string name, int a, int b) {
	cout<<name<<" "<<a*b<<endl;
	return a*b;
}

int main() {
	function<int ()> f = bind(test, "hello", 1, 2);
    auto g = bind(test, "hello", 1, 2);
    f();   // function 사용
    g();   // auto를 사용
}
