//#line 2 "/opt/ros/jade/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
// 
// File autogenerated for the bluefox_cam_node package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

#ifndef __bluefox_cam_node__BLUEFOX_CAM_NODECONFIG_H__
#define __bluefox_cam_node__BLUEFOX_CAM_NODECONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace bluefox_cam_node
{
  class bluefox_cam_nodeConfigStatics;
  
  class bluefox_cam_nodeConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(bluefox_cam_nodeConfig &config, const bluefox_cam_nodeConfig &max, const bluefox_cam_nodeConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const bluefox_cam_nodeConfig &config1, const bluefox_cam_nodeConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, bluefox_cam_nodeConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const bluefox_cam_nodeConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, bluefox_cam_nodeConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const bluefox_cam_nodeConfig &config) const = 0;
      virtual void getValue(const bluefox_cam_nodeConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T bluefox_cam_nodeConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (bluefox_cam_nodeConfig::* field);

      virtual void clamp(bluefox_cam_nodeConfig &config, const bluefox_cam_nodeConfig &max, const bluefox_cam_nodeConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const bluefox_cam_nodeConfig &config1, const bluefox_cam_nodeConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, bluefox_cam_nodeConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const bluefox_cam_nodeConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, bluefox_cam_nodeConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const bluefox_cam_nodeConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const bluefox_cam_nodeConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, bluefox_cam_nodeConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    template<class T, class PT>
    class GroupDescription : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string name, std::string type, int parent, int id, bool s, T PT::* f) : AbstractGroupDescription(name, type, parent, id, s), field(f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, bluefox_cam_nodeConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<bluefox_cam_nodeConfig::AbstractGroupDescriptionConstPtr> groups;
    };
    
class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(bluefox_cam_nodeConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("image_type"==(*_i)->name){image_type = boost::any_cast<int>(val);}
      }
    }

    int image_type;

    bool state;
    std::string name;

    
}groups;



//#line 280 "/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int image_type;
//#line 218 "/opt/ros/jade/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("bluefox_cam_nodeConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const bluefox_cam_nodeConfig &__max__ = __getMax__();
      const bluefox_cam_nodeConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const bluefox_cam_nodeConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const bluefox_cam_nodeConfig &__getDefault__();
    static const bluefox_cam_nodeConfig &__getMax__();
    static const bluefox_cam_nodeConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();
    
  private:
    static const bluefox_cam_nodeConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void bluefox_cam_nodeConfig::ParamDescription<std::string>::clamp(bluefox_cam_nodeConfig &config, const bluefox_cam_nodeConfig &max, const bluefox_cam_nodeConfig &min) const
  {
    return;
  }

  class bluefox_cam_nodeConfigStatics
  {
    friend class bluefox_cam_nodeConfig;
    
    bluefox_cam_nodeConfigStatics()
    {
bluefox_cam_nodeConfig::GroupDescription<bluefox_cam_nodeConfig::DEFAULT, bluefox_cam_nodeConfig> Default("Default", "", 0, 0, true, &bluefox_cam_nodeConfig::groups);
//#line 280 "/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.image_type = 0;
//#line 280 "/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.image_type = 2;
//#line 280 "/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.image_type = 0;
//#line 280 "/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(bluefox_cam_nodeConfig::AbstractParamDescriptionConstPtr(new bluefox_cam_nodeConfig::ParamDescription<int>("image_type", "int", 0, "image type enum", "{'enum_description': 'An enum to set the image type', 'enum': [{'srcline': 8, 'description': 'distorted image', 'srcfile': '/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'distorted'}, {'srcline': 9, 'description': 'undistorted image', 'srcfile': '/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'undistorted'}, {'srcline': 10, 'description': 'rectified image', 'srcfile': '/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'rectified'}]}", &bluefox_cam_nodeConfig::image_type)));
//#line 280 "/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(bluefox_cam_nodeConfig::AbstractParamDescriptionConstPtr(new bluefox_cam_nodeConfig::ParamDescription<int>("image_type", "int", 0, "image type enum", "{'enum_description': 'An enum to set the image type', 'enum': [{'srcline': 8, 'description': 'distorted image', 'srcfile': '/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'distorted'}, {'srcline': 9, 'description': 'undistorted image', 'srcfile': '/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'undistorted'}, {'srcline': 10, 'description': 'rectified image', 'srcfile': '/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'rectified'}]}", &bluefox_cam_nodeConfig::image_type)));
//#line 235 "/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 235 "/opt/ros/jade/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(bluefox_cam_nodeConfig::AbstractGroupDescriptionConstPtr(new bluefox_cam_nodeConfig::GroupDescription<bluefox_cam_nodeConfig::DEFAULT, bluefox_cam_nodeConfig>(Default)));
//#line 353 "/opt/ros/jade/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<bluefox_cam_nodeConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__); 
    }
    std::vector<bluefox_cam_nodeConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<bluefox_cam_nodeConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    bluefox_cam_nodeConfig __max__;
    bluefox_cam_nodeConfig __min__;
    bluefox_cam_nodeConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const bluefox_cam_nodeConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static bluefox_cam_nodeConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &bluefox_cam_nodeConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const bluefox_cam_nodeConfig &bluefox_cam_nodeConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const bluefox_cam_nodeConfig &bluefox_cam_nodeConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const bluefox_cam_nodeConfig &bluefox_cam_nodeConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<bluefox_cam_nodeConfig::AbstractParamDescriptionConstPtr> &bluefox_cam_nodeConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<bluefox_cam_nodeConfig::AbstractGroupDescriptionConstPtr> &bluefox_cam_nodeConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const bluefox_cam_nodeConfigStatics *bluefox_cam_nodeConfig::__get_statics__()
  {
    const static bluefox_cam_nodeConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = bluefox_cam_nodeConfigStatics::get_instance();
    
    return statics;
  }

//#line 8 "/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg"
      const int bluefox_cam_node_distorted = 0;
//#line 9 "/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg"
      const int bluefox_cam_node_undistorted = 1;
//#line 10 "/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/cfg/bluefox_cam_node.cfg"
      const int bluefox_cam_node_rectified = 2;
}

#endif // __BLUEFOX_CAM_NODERECONFIGURATOR_H__
