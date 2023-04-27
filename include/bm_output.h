#ifndef BM_OUTPUT_HPP
#define BM_OUTPUT_HPP

#include "bm_data.hpp"
namespace BM
{


	class service
	{
	public:
		service() {};
		~service() {};

		void set_vesselclassID  (int vesselclassID )
		{
			m_vesselclassID= vesselclassID;
		}

		const int get_vesselclassID() const
		{
			return m_vesselclassID;
		}

		void set_numVessels ( int numVessels )
		{
			m_numVessels= numVessels;
		}

		const int get_numVessels() const
		{
			return m_numVessels;
		}

		void set_frequency ( double frequency )
		{
			m_frequency= frequency;
		}

		const double get_frequency() const
		{
			return m_frequency;
		}

		void set_rotation ( std::vector<int> rotation )
		{
			m_rotation= rotation;
		}

		const std::vector<int> get_rotation() const
		{
			return m_rotation;
		}


	private:
		//VesselClass, should point to vesselclass in indata
		int m_vesselclassID;

		//Number of vessels
		int m_numVessels;

		//Frequency in days
		double m_frequency;

		//Rotation
		std::vector<int> m_rotation;


		//TODO's further extensions:
		//Speed at different Legs
		//Arrival and Departure times
		//VSA/SCA

	};

	struct portcall{
		int portcallId; //ports idx in indata.graph
		service* m_service;
		int num_portcall; //0-indexed, in case port X is called 3 times by m_service, this gives which.
	};

	class routing
	{
	public:
		routing() {};
		~routing() {};

		void set_volume ( double volume )
		{
			m_volume= volume;
		}

		const double get_volume() const
		{
			return m_volume;
		}

		void set_path ( std::vector<portcall> path )
		{
			m_path= path;
		}

		const std::vector<portcall> get_path() const
		{
			return m_path;
		}


	private:
		//Volume
		double m_volume;

		//Path
		std::vector<portcall> m_path;

	};


	class solution
	{
	public:
		solution() {};
		~solution() {};

		void set_indata ( BM::data* indata )
		{
			m_indata= indata;
		}

		 BM::data* get_indata()  
		{
			return m_indata;
		}

		void set_services ( std::vector<service> services )  
		{
			m_services= services;
		}
		std::vector<service> get_services()  
		{
			return m_services;
		}
		void set_routings ( std::vector<routing> routings )  
		{
			m_routings= routings;
		}
		std::vector<routing> get_routings() 
		{
			return m_routings;
		}


	private:
		//In Data
		BM::data* m_indata;

		//Services
		std::vector<service> m_services;

		//Routings
		std::vector<routing> m_routings;

	};



}
#endif // BM_OUTPUT_HPP