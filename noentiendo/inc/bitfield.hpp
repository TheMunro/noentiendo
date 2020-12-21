#pragma once

namespace noentiendo
{
template<typename enum_class_type>
class bitfield
{
	using underlying_type = std::underlying_type_t<enum_class_type>;
	
public:
	constexpr bitfield() noexcept
		: flags{}
	{
	}
	
	template <class U>
	constexpr bitfield(U initial_flags) noexcept
		: flags(static_cast<underlying_type>(initial_flags))
	{
	}

	//https://en.cppreference.com/w/cpp/named_req/BitmaskType
    constexpr bitfield operator~() const noexcept
	{
		return bitfield {~flags};
	}

	constexpr bitfield operator&(const bitfield& r) const noexcept
	{
		return bitfield {flags & r.flags};
	}

	constexpr bitfield operator|(const bitfield& r) const noexcept
	{
		return bitfield {flags | r.flags};
	}

	constexpr bitfield operator^(const bitfield& r) const noexcept
	{
		return bitfield {flags ^ r.flags};
	}

	constexpr bitfield& operator|=(const bitfield& r) noexcept
	{
		flags |= r.flags;
		return *this;
	}

	constexpr bitfield& operator&=(const bitfield& r) noexcept
	{
		flags &= r.flags;
		return *this;
	}

	constexpr bitfield& operator^=(const bitfield& r) noexcept
	{
		flags ^= r.flags;
		return *this;
	}

	//too lazy to add equality operators for now
	[[nodiscard]] constexpr enum_class_type value() const
	{
		return static_cast<enum_class_type>(flags);
	}

private:
	underlying_type flags;
};
}	 // namespace noentiendo