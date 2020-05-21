#ifndef __CONFIGURATION_H_
#define __CONFIGURATION_H_

#include <quadmap/quadmap.h>

// typedef quadmap::point2d        Configuration;
// typedef quadmap::QuadTreeKey    ConfigurationKey;

#if defined(__GNUC__) && ! defined(_LIBCPP_VERSION)
#include <tr1/unordered_set>
#include <tr1/unordered_map>
namespace unordered_ns = std::tr1;
#else
#include <unordered_set>
#include <unordered_map>
namespace unordered_ns = std;
#endif

#include <cmath>

struct Configuration {
    Configuration() { data[0] = data[1] = 0.0; }
    Configuration(const Configuration& _other) {
        data[0] = _other.x();
        data[1] = _other.y();
    }
    Configuration(float x, float y) {
        data[0] = x;
        data[1] = y;
    }
    inline Configuration& operator= (const Configuration& _other)  {
        data[0] = _other.x();
        data[1] = _other.y();
        return *this;
    }

    inline double& x() { return data[0]; }
    inline double& y() { return data[1]; }
    inline const double& x() const { return data[0]; }
    inline const double& y() const { return data[1]; }

    double data[2];
};

struct ConfigurationKey {
    ConfigurationKey() {
        k[0] = k[1] = 0;
    }
    ConfigurationKey(int a, int b){
        k[0] = a;
        k[1] = b;
    }

    ConfigurationKey(const ConfigurationKey& other){
        k[0] = other.k[0];
        k[1] = other.k[1];
    }

    bool operator== (const ConfigurationKey &other) const {
        return ((k[0] == other[0]) && (k[1] == other[1]));
    }

    bool operator!= (const ConfigurationKey& other) const {
        return ((k[0] != other[0]) || (k[1] != other[1]));
    }

    ConfigurationKey& operator=(const ConfigurationKey& other){
        k[0] = other.k[0]; k[1] = other.k[1];
        return *this;
    }

    const int& operator[] (unsigned int i) const {
        return k[i];
    }

    int& operator[] (unsigned int i) {
        return k[i];
    }

    int k[2];

    /// Provides a hash function on Keys
    struct KeyHash{
        std::size_t operator()(const ConfigurationKey& key) const{
            return static_cast<std::size_t>(key.k[0])
                   + 1447 * static_cast<std::size_t>(key.k[1]);
        }
    };

};

struct ConfigurationMap{
    ConfigurationMap(double _resolution) : resolution(_resolution), resolution_factor(1.0 / _resolution), grid_max_val(32768) {}

    quadmap::unordered_ns::unordered_map<ConfigurationKey, bool, ConfigurationKey::KeyHash> collision_hash;
    const unsigned int grid_max_val;
    const double resolution;
    const double resolution_factor;

    inline int coordToKey(double coordinate) const{
        return ((int)floor(resolution_factor * coordinate)) + grid_max_val;
    }
    inline ConfigurationKey coordToKey(const Configuration& _coord) const{
        return ConfigurationKey(coordToKey(_coord.x()), coordToKey(_coord.y()));
    }
    inline ConfigurationKey coordToKey(const double _x, const double _y) const{
        return ConfigurationKey(coordToKey(_x), coordToKey(_y));
    }

    inline double keyToCoord(int _key) const{
        return (double(_key - (int) this->grid_max_val) + 0.5) * this->resolution;
    }
    inline Configuration keyToCoord(const ConfigurationKey& _key) const{
        return Configuration(float(keyToCoord(_key[0])), float(keyToCoord(_key[1])));
    }
};

#endif