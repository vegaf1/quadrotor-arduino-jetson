# syntax: proto3
using ProtoBuf
import ProtoBuf.meta

mutable struct PWM <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function PWM(; kwargs...)
        obj = new(meta(PWM), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct PWM
const __meta_PWM = Ref{ProtoMeta}()
function meta(::Type{PWM})
    ProtoBuf.metalock() do
        if !isassigned(__meta_PWM)
            __meta_PWM[] = target = ProtoMeta(PWM)
            allflds = Pair{Symbol,Union{Type,String}}[:motor1 => Float64, :motor2 => Float64, :motor3 => Float64, :motor4 => Float64]
            meta(target, PWM, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_PWM[]
    end
end
function Base.getproperty(obj::PWM, name::Symbol)
    if name === :motor1
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :motor2
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :motor3
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :motor4
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

export PWM
