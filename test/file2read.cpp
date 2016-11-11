#include <fstream>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    float x, y, z, r;
    ifstream in("out.txt", ios::in);
    if (!in.is_open()) {
        cout << "Error opening file";
        return 1;
    }
    in >> x >> y >> z >> r;
    cout << "x: " << x << "y: " << y << "z: " << z << "r: " << r << endl;
    in.close();

    return 0;
}
