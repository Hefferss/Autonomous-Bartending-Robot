import os
from math import pi
from spatialgeometry import Mesh
from spatialmath import SE3
from spatialmath.base import troty, trotz
from ir_support_extra_parts.parts import part_mesh #estop and safety man from A1

#all the bar objects from MainRobotSimFile.py but class (object orientated):) each one is kept as self.something so the robots can grab them later
class BarScene:
    def __init__(self, env): #env has to be launched already, so make this AFTER workcell.launch()
        item_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Scene_parts")

        #cognac scene load
        cognac_location = os.path.join(item_folder, "Cognac.stl")
        self.cognac_scene = Mesh(filename=cognac_location, color="#522B0F", scale=[0.001, 0.001, 0.001])
        self.cognac_scene.T = SE3(0.3, -0.25, 0.375) 
        env.add(self.cognac_scene)

        #cointreau.stl
        cointreau_location = os.path.join(item_folder, "Cointreau.stl")
        self.cointreau_scene = Mesh(filename=cointreau_location, color="#C47C07", scale=[0.02, 0.02, 0.02])
        self.cointreau_scene.T = SE3(0.3, -0.2, 0.25)
        env.add(self.cointreau_scene)

        #lemonSlice.stl 
        lemon_location = os.path.join(item_folder, "Lemon.stl")
        self.lemon_scene = Mesh(filename=lemon_location, color="#F7F7A1")
        self.lemon_scene.T = SE3(0.22, -0.2, 0.26) * troty(pi/2)
        env.add(self.lemon_scene)

        #BarTableRobot.stl
        bar_table_location = os.path.join(item_folder, "BarTableRobot.stl")
        self.bar_table_scene = Mesh(filename=bar_table_location, color="#8B4513", scale=[0.001, 0.0014, 0.001]) # y stretched 1.4x so the front reaches the drop off zones
        self.bar_table_scene.T = SE3(-1.5, 0.3, 0) # front edge moved from y 0.5 to 0.3, back edge stays at 1.0
        env.add(self.bar_table_scene)

        #SensorsTable
        sensors_table_location = os.path.join(item_folder, "SensorsTable.stl")
        self.sensors_table_scene = Mesh(filename=sensors_table_location, color="#8B4513", scale=[0.001, 0.001, 0.001])
        self.sensors_table_scene.T = SE3(1.85,0,0) * trotz(pi/2) * SE3(0,0.25,0)
        env.add(self.sensors_table_scene)

        #BarTableStorage.stl
        bar_table_storage_location = os.path.join(item_folder, "BarTableStorage.stl")
        self.bar_table_storage_scene = Mesh(filename=bar_table_storage_location, color="#8B4513", scale=[0.001, 0.001, 0.001])
        self.bar_table_storage_scene.T = SE3(-0.5, -0.5, 0) 
        env.add(self.bar_table_storage_scene)

        #BetterLemon.stl
        Better_lemon_location = os.path.join(item_folder, "BetterLemon.stl")
        self.Better_lemon_scene = Mesh(filename=Better_lemon_location, color="#F7F7A1", scale=[0.001, 0.001, 0.001])
        self.Better_lemon_scene.T = SE3(0.2, -0.2, 0.26) * troty(pi/2)
        env.add(self.Better_lemon_scene)

        #UR3 rail
        ur3_rail_location = os.path.join(item_folder, "RailForUR3.stl")
        self.ur3_rail_scene = Mesh(filename=ur3_rail_location, color="#C0C0C0", scale=[0.001, 0.001, 0.001])
        self.ur3_rail_scene.T = SE3(-1.5, 0.1, 0)
        env.add(self.ur3_rail_scene)

        #cocktail glass
        cocktail_glass_location = os.path.join(item_folder, "CocktailGlass.stl")
        self.cocktail_glass_scene = Mesh(filename=cocktail_glass_location, color="#FFFFFF", scale=[0.001, 0.001, 0.001])
        self.cocktail_glass_scene.T = SE3(-0.2, -0.1, 0.255)
        env.add(self.cocktail_glass_scene)

        #cocktail shaker
        cocktail_shaker_location = os.path.join(item_folder, "CocktailShaker.stl")
        self.cocktail_shaker_scene = Mesh(filename=cocktail_shaker_location, color="#504E4E", scale=[0.001, 0.001, 0.001])
        self.cocktail_shaker_scene.T = SE3(0, -0.2, 0.255)
        env.add(self.cocktail_shaker_scene)

        #Umbrella pot
        Umbrella_pot_location = os.path.join(item_folder, "UmbrellaPot.stl")
        self.Umbrella_pot_scene = Mesh(filename=Umbrella_pot_location, color="#504E4E", scale=[0.001, 0.001, 0.001])
        self.Umbrella_pot_scene.T = SE3(-0.2, -0.3, 0.255)
        env.add(self.Umbrella_pot_scene)

        #Umbrella 
        Umbrella_location = os.path.join(item_folder, "Umbrella.stl")
        self.Umbrella_scene = Mesh(filename=Umbrella_location, color="#F601C1", scale=[0.001, 0.001, 0.001])
        self.Umbrella_scene.T = SE3(-0.2, -0.3, 0.255) * trotz(pi/2)
        env.add(self.Umbrella_scene)

        #lemon pot
        Lemon_pot_location = os.path.join(item_folder, "UmbrellaPot.stl")
        self.Lemon_pot_scene = Mesh(filename=Lemon_pot_location, color="#504E4E", scale=[0.001, 0.001, 0.001])
        self.Lemon_pot_scene.T = SE3(0.2, -0.2, 0.26)
        env.add(self.Lemon_pot_scene)

        #cocktail shaker lid, STL is modelled sitting on the shaker so drop it 0.21 to sit on the table
        cocktail_shaker_lid_location = os.path.join(item_folder, "CocktailShakerLid.stl")
        self.cocktail_shaker_lid_scene = Mesh(filename=cocktail_shaker_lid_location, color="#504E4E", scale=[0.001, 0.001, 0.001])
        self.cocktail_shaker_lid_scene.T = SE3(0.7, -0.25, 0.045)
        env.add(self.cocktail_shaker_lid_scene)

        #safety tape (1m long pieces) - taped box from x -2 to 3, y -1 to 2
        safety_tape_location = os.path.join(item_folder, "SafetyTape.stl")
        tape_poses = [SE3(-2, -1, 0), SE3(-1, -1, 0), SE3(0, -1, 0), SE3(1, -1, 0), SE3(2, -1, 0),
                      SE3(-2, 1.9, 0), SE3(-1, 1.9, 0), SE3(0, 1.9, 0), SE3(1, 1.9, 0), SE3(2, 1.9, 0),
                      SE3(-2, 0, 0) * trotz(-pi/2), SE3(-2, 1, 0) * trotz(-pi/2), SE3(-2, 2, 0) * trotz(-pi/2),
                      SE3(2.9, 0, 0) * trotz(-pi/2), SE3(2.9, 1, 0) * trotz(-pi/2), SE3(2.9, 2, 0) * trotz(-pi/2)]
        self.safety_tapes = []
        for pose in tape_poses:
            tape = Mesh(filename=safety_tape_location, color="#F2C200", scale=[0.001, 0.001, 0.001])
            tape.T = pose
            env.add(tape)
            self.safety_tapes.append(tape)

        #Estop, just outside the front tape
        self.estop = part_mesh("EmergencyStopButton")
        self.estop.T = SE3(-1.5, 2.1, 0)
        env.add(self.estop)

        #Safety Man, standing next to the estop
        self.safety_man = part_mesh("SafetyPerson")
        self.safety_man.T = SE3(-1.0, 2.4, 0)
        env.add(self.safety_man)