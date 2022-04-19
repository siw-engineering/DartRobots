#include "PerlinNoise.h"
#include <cmath>
#include <algorithm>
#include <random>

PerlinNoise::PerlinNoise() {

    p.insert(p.end(), p.begin(), p.end());
}

PerlinNoise::PerlinNoise(int seed) {

    std::shuffle(std::begin(p), std::end(p), std::default_random_engine(seed));
    p.insert(p.end(), p.begin(), p.end());
}

float PerlinNoise::noise(float x, float y, float z) {

    // Find the unit cube that contains the point
    int X = (int) floor(x) & 255;
    int Y = (int) floor(y) & 255;
    int Z = (int) floor(z) & 255;

    // Find relative x, y,z of point in cube
    x -= floor(x);
    y -= floor(y);
    z -= floor(z);

    // Compute fade curves for each of x, y, z
    float u = fade(x);
    float v = fade(y);
    float w = fade(z);

    // Hash coordinates of the 8 cube corners
    int A = p[X] + Y;
    int AA = p[A] + Z;
    int AB = p[A + 1] + Z;
    int B = p[X + 1] + Y;
    int BA = p[B] + Z;
    int BB = p[B + 1] + Z;

    // Add interpolated results from 8 corners of cube
    float res = lerp(w,
                     lerp(v,
                          lerp(u, grad(p[AA], x, y, z), grad(p[BA], x-1, y, z)),
                          lerp(u, grad(p[AB], x, y-1, z), grad(p[BB], x-1, y-1, z))),
                     lerp(v,
                          lerp(u, grad(p[AA+1], x, y, z-1), grad(p[BA+1], x-1, y, z-1)),
                          lerp(u, grad(p[AB+1], x, y-1, z-1),grad(p[BB+1], x-1, y-1, z-1))));
    return (res + 1.0)/2.0;
}

float PerlinNoise::fade(float t) { 
    return t * t * t * (t * (t * 6 - 15) + 10);
}

float PerlinNoise::lerp(float t, float a, float b) { 
    return a + t * (b - a);
}

float PerlinNoise::grad(int hash, float x, float y, float z) {
    int h = hash & 15;
    // Convert lower 4 bits of hash into 12 gradient directions
    float u = h < 8 ? x : y,
          v = h < 4 ? y : h == 12 || h == 14 ? x : z;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}