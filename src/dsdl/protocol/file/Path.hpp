/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /mnt/c/Users/BrianTaylor/Documents/Software/dronecan/dsdl/uavcan/protocol/file/Path.uavcan
 */

#ifndef UAVCAN_PROTOCOL_FILE_PATH_HPP_INCLUDED
#define UAVCAN_PROTOCOL_FILE_PATH_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Nested type.
#
# File system path in UTF8.
#
# The only valid separator is forward slash.
#

uint8 SEPARATOR = '/'

uint8[<=200] path
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.file.Path
saturated uint8[<=200] path
******************************************************************************/

#undef path
#undef SEPARATOR

namespace uavcan
{
namespace protocol
{
namespace file
{

template <int _tmpl>
struct UAVCAN_EXPORT Path_
{
    typedef const Path_<_tmpl>& ParameterType;
    typedef Path_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > SEPARATOR;
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 200 > path;
    };

    enum
    {
        MinBitLen
            = FieldTypes::path::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::path::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::SEPARATOR >::Type SEPARATOR; // '/'

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::path >::Type path;

    Path_()
        : path()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<1608 == MaxBitLen>::check();
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
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.file.Path";
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
bool Path_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        path == rhs.path;
}

template <int _tmpl>
bool Path_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(path, rhs.path);
}

template <int _tmpl>
int Path_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::path::encode(self.path, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Path_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::path::decode(self.path, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Path_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x12AEFC50878A43E2ULL);

    FieldTypes::path::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename Path_<_tmpl>::ConstantTypes::SEPARATOR >::Type
    Path_<_tmpl>::SEPARATOR = 47U; // '/'

/*
 * Final typedef
 */
typedef Path_<0> Path;

// No default registration

} // Namespace file
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::file::Path >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::file::Path::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::file::Path >::stream(Stream& s, ::uavcan::protocol::file::Path::ParameterType obj, const int level)
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
    s << "path: ";
    YamlStreamer< ::uavcan::protocol::file::Path::FieldTypes::path >::stream(s, obj.path, level + 1);
}

}

namespace uavcan
{
namespace protocol
{
namespace file
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::file::Path::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::file::Path >::stream(s, obj, 0);
    return s;
}

} // Namespace file
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_FILE_PATH_HPP_INCLUDED