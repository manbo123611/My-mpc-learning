#include<bits/stdc++.h>
using namespace std;

int main()
{
	ios::sync_with_stdio(false);
	cin.tie(nullptr);
	int n;
	cin >> n;
	vector<long long> ordinary(n),banker(n);
	for (int i = 0; i < n; ++i)
	{
		string s;
		cin >> s;
		size_t dot = s.find('.');
		long long a = stoll(s.substr(0,dot));
		int b = s[dot + 1] - '0';
		ordinary[i] = a + (b >= 5);
		banker[i] = a + (b > 5 || (b == 5 && (a & 1LL)));
	}
	for (int i = 0; i < n; ++i)
		cout << (i ? " " : "") << ordinary[i];
	cout << '\n';
	for (int i = 0; i < n; ++i)
		cout << (i ? " " : "") << banker[i];
	cout << '\n';
	return 0;
}
