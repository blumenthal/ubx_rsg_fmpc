
local ubx=require "ubx"
local ffi = require("ffi")


local coordinated_block

function init(block)
   print("init")

   --ubx.config_add(block, "trig_blocks", "Blocks that will be toggeled on/off.", "struct ptrig_config", 1) -- this one does NOT work
   --ubx.config_add(block, "test", "Just a test", "int", 1)  
   ubx.port_add(block, "motion_toggler", "Input data port that toggles the controller on/off.", "unsigned char", 1, nil, 0, 0)
   ubx.port_add(block, "youbot_cmd_mode", "Output port to toggel mode of youbot between OFF and VELOCITY", "int32_t", 1, "int32_t", 1, 0)

   return true
end

function start(block)
  print("motion_coordinator.lua: start")
  print(block)
  return true
--  local r = ubx.resolve_types(block)
--
--  local ptab_before = ubx.ports_map(block, ubx.port_totab)
--
--  -- PROBLEM it is not possible to read the below config file :-(!
--
--  local bt=ubx.block_totab(block) 
--  -- results in: ERR call_hook: motion_coordinator: error calling function start: ...microblx/lua/ubx.lua:318: attempt to index local 'b' (a userdata value)
-- 
--  local config = ubx.config_get_data(block, "trig_blocks") -- cannot be passed as .usc configuration parameter?
--  --local trig_blocks = ubx.data_tolua(ubx.config_get_data(block, "trig_blocks"))
--  
--  local cb =  ubx.data_tolua(config)
--  -- ERR call_hook: motion_coordinator: error calling function start: .../microblx/lua/ubx.lua:497: undeclared or implicit tag 'ptrig_config'
--  
--  print(cb) 
--    
--  coordinated_block = cb.b
--
--
--  local test =  ubx.config_get(block, "test")
--  print(ubx.config_tostr(test))
--
--  return true
end

function step(block)

  -- read data at port
  local res, data = ubx.port_read(ubx.port_get(block, "motion_toggler"))
  print("motion_coordinator.lua: res = " .. res .. " data = " .. ubx.data_tostr(data));
  
  if res > 0 then
    print ("\tNew toggle command. Setting to: " .. ubx.data_tolua(data))
    
    -- handle the "protocol"
    if ubx.data_tolua(data) == 0 then
      print("\t Motion = OFF")
      --ubx.block_stop(coordinated_block)
      ubx.port_write(ubx.port_get(block, "youbot_cmd_mode"), 0)
    elseif ubx.data_tolua(data) == 1 then
      print("\t Motion = ON")
      --ubx.block_start(coordinated_block)
      ubx.port_write(ubx.port_get(block, "youbot_cmd_mode"), 2)
    else
      print("Unknown command.")
    end
    
  end

end

function stop(block)
   print("motion_coordinator.lua: stop")

end

function cleanup(block) 
   print("motion_coordinator.lua: cleanup")
   print("rm'ing", ubx.port_rm(block, "motion_toggler"))
   print("rm'ing", ubx.port_rm(block, "youbot_cmd_mode"))
   --print("rm'ing", ubx.config_rm(block, "test"))   
end
