#include<iostream>
#include<vector>
using namespace std;
template <typename T>
void computeAllChoices(std::vector<T>& data, int n, int outLen, int startIndex, int m, int* arr, int arrIndex)
{
    if (m == 0)
    {
        for (int i = 0; i < outLen; i++)
        {
            std::cout << arr[i] << "\t";
        }
        std::cout << std::endl;

        return;
    }

    int endIndex = n - m;
    for (int i = startIndex; i <= endIndex; i++)
    {
        arr[arrIndex] = data[i];
        computeAllChoices(data, n, outLen, i + 1, m - 1, arr, arrIndex + 1);
    }
}

int main(int argc, char* argv[])
{
    std::vector<int> data;
    for (int i = 0; i < 8; i++)
    {
        data.push_back(i + 1);
    }

    int arr[5];

    computeAllChoices(data, data.size(), 5, 0, 5, arr, 0);
    system("pause");
    return 0;
}