#include <fstream>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    float x=5.1, y=6.2, z=7.3, r=8.4;
    ofstream out("out.txt");
    if (out.is_open()) {
        out << x << '\t' << y << '\t' << z << '\t' << r << endl;
        out.close();
    }
    return 0;
}
