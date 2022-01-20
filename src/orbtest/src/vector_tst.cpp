#include <iostream>
#include <cstdio>
#include <vector>

using namespace std;

int main(int argc, char** argv){
    vector<int> aa = {10, 20, 30, 40, 50};
    int num = 0;
    int cnt = 0;

    for (vector<int>::iterator i = aa.begin(); i != aa.end(); ++i){
        cout << ++num << " times Address : ";
        cout << &i << " value : ";
        cout << *i << " value Address : ";
        cout << &(*i) << endl;

        if ((*i) == 20){
            i = aa.erase(i);
            // --(i = aa.erase(i));
        }
    }
    cout << endl << "show the result" << endl << endl;

    for (vector<int>::iterator i = aa.begin(); i != aa.end(); ++i){
        cout << ++cnt << " times Address : ";
        cout << &i << " value : ";
        cout << *i << " value Address : ";
        cout << &(*i) << endl;
    }

    return 0;
}