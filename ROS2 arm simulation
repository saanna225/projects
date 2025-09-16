import rclpy

from rclpy.node import Node

from xarmrob_interfaces.msg import ME439PointXYZ

from std_msgs.msg import Float32

import numpy as np





class TowerOfHanoi(Node):

    def __init__(self):

        super().__init__('game_operation')



        self.xyz_goal = [0.2, 0.0, 0.25]  # Initial neutral position

        self.gripper_angle = 0.0  # Initial gripper angle (open)



        # Publishers

        self.endpoint_publisher = self.create_publisher(ME439PointXYZ, '/hanoi_endpoints', 1)

        self.gripper_angle_publisher = self.create_publisher(Float32, '/gripper_angle', 1)



        self.endpoint_desired_msg = ME439PointXYZ()

        self.endpoint_desired_msg.xyz = self.xyz_goal



        # Define rod positions

        self.rod_a = [0.2, 0.1]

        self.rod_b = [0.2, 0.0]

        self.rod_c = [0.2, -0.1]



        # Rod and disc parameters

        self.disc_height = 0.025

        self.num_discs = 3

        self.clearance_height = 0.25

        self.base_height = 0.15



        # Rod state

        self.rods = self.define_rods()



        # Step tracking for `move_with_clearance_exe`

        self.current_step = 0

        self.current_move = None



        # Start the algorithm

        self.moves = []

        self.tower_of_hanoi_algorithm(self.num_discs, 'A', 'C', 'B')

        self.get_logger().info(f"Total moves planned: {len(self.moves)}")



        # Start the move execution

        self.timer = self.create_timer(2.0, self.execute_moves)



    def define_rods(self):

        return {

            'A': [3, 2, 1],  # Discs on Rod A (disc 3 at the bottom, disc 1 at the top)

            'B': [],

            'C': [],

        }



    def send_endpoint_desired(self):

        self.endpoint_desired_msg.xyz = self.xyz_goal

        self.endpoint_publisher.publish(self.endpoint_desired_msg)

        self.get_logger().info(f"Publishing endpoint: {self.xyz_goal}")



    def send_gripper_angle(self):

        msg = Float32()

        msg.data = float(self.gripper_angle)

        self.gripper_angle_publisher.publish(msg)

        self.get_logger().info(f"Publishing gripper angle: {self.gripper_angle}")



    def move_xyz(self, x, y, z):

        self.xyz_goal[0] = float(x)

        self.xyz_goal[1] = float(y)

        self.xyz_goal[2] = float(z)

        self.send_endpoint_desired()



    def tower_of_hanoi_algorithm(self, n, source, target, auxiliary):

        if n == 0:

            return



        # Move n-1 discs from source to auxiliary

        self.tower_of_hanoi_algorithm(n - 1, source, auxiliary, target)



        # Plan the move for the nth disc

        self.moves.append((source, target))



        # Move the n-1 discs from auxiliary to target

        self.tower_of_hanoi_algorithm(n - 1, auxiliary, target, source)



    def execute_moves(self):

        if self.current_move is None and self.moves:

            # Get the next move

            source, target = self.moves.pop(0)

            self.current_move = (source, target)



            # Calculate positions

            source_pos = self.get_rod_position(source)

            target_pos = self.get_rod_position(target)



            disc_pos = (len(self.rods[source]) * self.disc_height) + (self.disc_height / 2) + self.base_height

            self.current_steps = [

                (source_pos[0], source_pos[1], self.clearance_height),  # Step 1: lift

                (source_pos[0], source_pos[1], disc_pos),               # Step 2: lower to pick up

                "gripper_angle",  # Step 3: set gripper angle

                (target_pos[0], target_pos[1], self.clearance_height),  # Step 4: lift to clearance

                (target_pos[0], target_pos[1], self.clearance_height),  # Step 5: move to target rod

                "gripper_angle_release",  # Step 6: release gripper

            ]

            self.current_step = 0



        # Execute current move step by step

        if self.current_move and self.current_step < len(self.current_steps):

            step = self.current_steps[self.current_step]

            self.get_logger().info(f"Executing Step {self.current_step + 1}: {step}")  # Log current step


            if isinstance(step, tuple):

                # Move XYZ

                x, y, z = step

                self.move_xyz(x, y, z)

            elif step == "gripper_angle":

                # Set gripper angle to pick up (e.g., np.pi / 4)

                self.gripper_angle = np.pi / 4

                self.send_gripper_angle()

            elif step == "gripper_angle_release":

                # Set gripper angle to release (e.g., 0)

                self.gripper_angle = 0

                self.send_gripper_angle()



            self.current_step += 1





        elif self.current_move:

            # Finalize move and release disc

            source, target = self.current_move

            disc = self.rods[source].pop()

            self.rods[target].append(disc)

            self.get_logger().info(f"Moved disc {disc} from {source} to {target}")

            self.current_move = None



        # Stop timer if all moves are complete

        if not self.moves and self.current_move is None:

            self.get_logger().info("Tower of Hanoi complete!")

            self.timer.cancel()



    def get_rod_position(self, rod):

        if rod == 'A':

            return [self.rod_a[0], self.rod_a[1], 0.0]

        elif rod == 'B':

            return [self.rod_b[0], self.rod_b[1], 0.0]

        elif rod == 'C':

            return [self.rod_c[0], self.rod_c[1], 0.0]





def main(args=None):

    rclpy.init(args=args)

    node = TowerOfHanoi()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()





if __name__ == '__main__':

    main()

