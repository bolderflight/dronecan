/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /mnt/c/Users/BrianTaylor/Documents/Software/dronecan/dsdl/uavcan/equipment/air_data/1021.IndicatedAirspeed.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# IAS.
#

float16 indicated_airspeed              # m/s
float16 indicated_airspeed_variance     # (m/s)^2
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.air_data.IndicatedAirspeed
saturated float16 indicated_airspeed
saturated float16 indicated_airspeed_variance
******************************************************************************/

#undef indicated_airspeed
#undef indicated_airspeed_variance

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <int _tmpl>
struct UAVCAN_EXPORT IndicatedAirspeed_
{
    typedef const IndicatedAirspeed_<_tmpl>& ParameterType;
    typedef IndicatedAirspeed_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > indicated_airspeed;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > indicated_airspeed_variance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::indicated_airspeed::MinBitLen
            + FieldTypes::indicated_airspeed_variance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::indicated_airspeed::MaxBitLen
            + FieldTypes::indicated_airspeed_variance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::indicated_airspeed >::Type indicated_airspeed;
    typename ::uavcan::StorageType< typename FieldTypes::indicated_airspeed_variance >::Type indicated_airspeed_variance;

    IndicatedAirspeed_()
        : indicated_airspeed()
        , indicated_airspeed_variance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<32 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1021 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.air_data.IndicatedAirspeed";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool IndicatedAirspeed_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        indicated_airspeed == rhs.indicated_airspeed &&
        indicated_airspeed_variance == rhs.indicated_airspeed_variance;
}

template <int _tmpl>
bool IndicatedAirspeed_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(indicated_airspeed, rhs.indicated_airspeed) &&
        ::uavcan::areClose(indicated_airspeed_variance, rhs.indicated_airspeed_variance);
}

template <int _tmpl>
int IndicatedAirspeed_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::indicated_airspeed::encode(self.indicated_airspeed, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::indicated_airspeed_variance::encode(self.indicated_airspeed_variance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int IndicatedAirspeed_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::indicated_airspeed::decode(self.indicated_airspeed, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::indicated_airspeed_variance::decode(self.indicated_airspeed_variance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature IndicatedAirspeed_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xA1892D72AB8945FULL);

    FieldTypes::indicated_airspeed::extendDataTypeSignature(signature);
    FieldTypes::indicated_airspeed_variance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef IndicatedAirspeed_<0> IndicatedAirspeed;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::air_data::IndicatedAirspeed > _uavcan_gdtr_registrator_IndicatedAirspeed;

}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::air_data::IndicatedAirspeed >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::air_data::IndicatedAirspeed::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::air_data::IndicatedAirspeed >::stream(Stream& s, ::uavcan::equipment::air_data::IndicatedAirspeed::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "indicated_airspeed: ";
    YamlStreamer< ::uavcan::equipment::air_data::IndicatedAirspeed::FieldTypes::indicated_airspeed >::stream(s, obj.indicated_airspeed, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "indicated_airspeed_variance: ";
    YamlStreamer< ::uavcan::equipment::air_data::IndicatedAirspeed::FieldTypes::indicated_airspeed_variance >::stream(s, obj.indicated_airspeed_variance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::air_data::IndicatedAirspeed::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::air_data::IndicatedAirspeed >::stream(s, obj, 0);
    return s;
}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_AIR_DATA_INDICATEDAIRSPEED_HPP_INCLUDED