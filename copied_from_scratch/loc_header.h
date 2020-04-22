#ifndef LOCATION_HEADER_H
#define LOCATION_HEADER_H

#include "ns3/nstime.h"
#include "ns3/header.h"
#include "ns3/vector.h"


class loc_header : public Header 
{
	public:
        void SetLocation(const Vector &location);
        const Vector GetLocation() const;// const Vector & GetPosition () const; the meaning of & here?
		static TypeId GetTypeId(void);
		//virtual TypeId GetInstanceTypeId (void) const;
		uint32_t GetSerializedSize (void) const;
		void Serialize (Buffer:Iterator start) const;
        void Deserialize (Buffer::Iterator start) const; 
		//virtual void Print (std::ostream &os) const;
			
	private:
		uint32_t m_data;
}

NS_OBJECT_ENSURE_REGISTERED (MyHeader);

