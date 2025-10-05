#ifndef FCB_GEO_FENCE_BASE_H_
#define FCB_GEO_FENCE_BASE_H_


#include "../helpers/gps.hpp"


#include "../de_common/helpers/json_nlohmann.hpp"
using Json_de = nlohmann::json;



namespace de
{
namespace fcb
{
namespace geofence
{

    
    enum ENUM_GEOFENCE_TYPE
    {
        LinearFence     = 1,
        PolygonFence    = 2,
        CylindersFence  = 3
    };

    class CGeoFenceBase
    {
        public:

            CGeoFenceBase ();
            ~CGeoFenceBase();

        public:
            virtual void parse (const Json_de& message);
            virtual double isInside(double lat, double lng, double alt) const {
                // Provide a default implementation here
                // This can be a simple implementation that always returns 0.0
                // or a more complex implementation that throws an exception
                return 0.0;
            }
            
        public:

            virtual Json_de getMessage();

            inline std::string getName ()
            {
                return m_fence_name;
            }

            inline bool shouldKeepOutside ()
            {
                return m_should_keep_outside;
            }

            inline int hardFenceAction ()
            {
                return m_hard_fence_action;
            }

            inline int getType()
            {
                return m_geofence_type;
            }

            

        protected:

            ENUM_GEOFENCE_TYPE m_geofence_type;
            std::string m_fence_name;
            bool m_should_keep_outside;
            int m_hard_fence_action;

            /**
            * @brief  in degrees
            * 
            */
            double m_latitude = 0;

            /**
            * @brief  in degrees
            * 
            */
            double m_longitude = 0;


            /**
             * @brief in meters
             * 
             */
            double m_altitude = 0;

            /**
             * @brief Location in message is NOT E7 format. but the object uses E7.
             * 
             */
            Json_de m_message;
            
    };


    class CGeoFenceCylinder :public CGeoFenceBase
    {
        public:

            CGeoFenceCylinder ();
            ~CGeoFenceCylinder();

        public:
        
            void parse (const Json_de& message)override;
            double isInside(double lat, double lng, double alt) const override;
            Json_de getMessage() override;
            
            inline void getLocation (double& lat, double& lng, double& alt) const
            {
                lat = m_latitude;
                lng = m_longitude;
                alt = m_altitude;
            }

            inline int getRadius()
            {
                return m_radius;
            }

        protected:
            /**
             * @brief in meters
             * 
             */
            int m_radius;

    };


    class CGeoFencePolygon :public CGeoFenceBase
    {
        public:

            CGeoFencePolygon ();
            ~CGeoFencePolygon();

        public:
        
            void parse (const Json_de& message)override;
            double isInside(double lat, double lng, double alt) const override;
            Json_de getMessage() override;
        
        protected:
            std::vector<POINT_3D> m_vertex;
    };


    class CGeoFenceLine :public CGeoFenceBase
    {
        public:

            CGeoFenceLine ();
            ~CGeoFenceLine();

        public:
        
            void parse (const Json_de& message)override;
            double isInside(double lat, double lng, double alt) const override;
            Json_de getMessage() override;

        protected:
            std::vector<POINT_3D> m_vertex;
            /**
             * @brief in meters
             * 
             */
            int m_width;
    };


    class CGeoFenceFactory
    {
        public:

            
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CGeoFenceFactory& getInstance()
            {
                static CGeoFenceFactory instance;

                return instance;
            }

            CGeoFenceFactory(CGeoFenceFactory const&)            = delete;
            void operator=(CGeoFenceFactory const&)              = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

            private:

                CGeoFenceFactory() 
                {
                }

                
            public:
                
                ~CGeoFenceFactory ()
                {

                }


            public:

                std::unique_ptr<de::fcb::geofence::CGeoFenceBase> getGeoFenceObject (const Json_de& message) const;
            

    };

    
}
}
}

#endif
