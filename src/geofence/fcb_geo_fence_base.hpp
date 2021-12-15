#ifndef FCB_GEO_FENCE_BASE_H_
#define FCB_GEO_FENCE_BASE_H_


#include "../helpers/json.hpp"
using Json = nlohmann::json;


namespace uavos
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
            virtual void parse (const Json& message);
            virtual void evaluate(double lat, double lng, double alt){ };

        public:
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


        protected:

            ENUM_GEOFENCE_TYPE m_geofence_type;
            std::string m_fence_name;
            bool m_should_keep_outside;
            int m_hard_fence_action;

            /**
            * @brief  in notmal -90,90
            * 
            */
            double m_latitude = 0;

            /**
            * @brief  in notmal 180-180
            * 
            */
            double m_longitude = 0;


            /**
             * @brief in meters
             * 
             */
            double m_altitude = 0;

    };


    class CGeoFenceCylinder :public CGeoFenceBase
    {
        public:

            CGeoFenceCylinder ();
            ~CGeoFenceCylinder();

        public:
        
            void parse (const Json& message)override;
            void evaluate(double lat, double lng, double alt) override;
        
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
        
            void parse (const Json& message)override;
            void evaluate(double lat, double lng, double alt) override;
        
    };


    class CGeoFenceLine :public CGeoFenceBase
    {
        public:

            CGeoFenceLine ();
            ~CGeoFenceLine();

        public:
        
            void parse (const Json& message)override;
            void evaluate(double lat, double lng, double alt) override;
        
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

                std::unique_ptr<uavos::fcb::geofence::CGeoFenceBase> getGeoFenceObject (const Json& message) const;
            

    };

    
}
}
}

#endif
