/*   Convert this program to C++
 *   change to C++ io
 *   change to one line comments
 *   change defines of constants to const
 *   change array to vector<>
 *   inline any short function
 */
#include "../include/sum.h"
#include <iostream>
using namespace std;
const int SIZE = 40;

template <class summable>
inline void sum(int &result, int size, const summable data[])
{
  // Sum all elements of the data array
  result = 0;
  for (int idx = 0; idx < size; ++idx)
    result = result + data[idx];
}

int main()
{
  int accum = 0;
  int data[SIZE];
  for (int idx = 0; idx < SIZE; ++idx)
    data[idx] = idx;
  sum(accum, SIZE, data);
  cout << "sum is " << accum << endl;
  return 0;
}