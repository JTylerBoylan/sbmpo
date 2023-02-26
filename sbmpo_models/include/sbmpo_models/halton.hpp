#ifndef SBMPO_MODELS_HALTON_HPP
#define SBMPO_MODELS_HALTON_HPP

#include <vector>
#include <cmath>

std::vector<double> halton(int n, int base)
{
    std::vector<double> v(n);
    double factor = 1.0 / base;
    int i = n;
    while (i > 0)
    {
        v[--i] = fmod(n, base) * factor;
        n /= base;
        factor /= base;
    }
    return v;
}

std::vector<std::vector<double>> generate_halton_samples(int N, int num_samples)
{
    std::vector<int> primes = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29}; // first 10 prime numbers
    std::vector<std::vector<double>> samples(num_samples, std::vector<double>(N));
    for (int i = 0; i < num_samples; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            samples[i][j] = halton(i + 1, primes[j]);
        }
    }
    return samples;
}

#endif