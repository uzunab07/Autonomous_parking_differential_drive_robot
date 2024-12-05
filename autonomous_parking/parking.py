class ParkingLot:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        # initialize grid with lanes which allow movement in all directions
        self.grid = [[{"type": "lane", "directions": {'N', 'E', 'S', 'W'}} for _ in range(cols)] for _ in range(rows)]

    # set a cell to be a parking spot with an entrance in a particular direction (because parking spots have lines enclosing 3 sides)
    def set_parking_spot(self, row, col, entrance):
        if entrance not in {'N', 'E', 'S', 'W'}:
            print("Invalid entrance direction. Must be 'N', 'E', 'S', or 'W'.")
            return
        
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.grid[row][col] = {
                "type": "parking_spot",
                "entrance": entrance
            }
        else:
            print("Invalid parking spot coordinates.")

    # set allowed movement directions for a lane
    def set_lane_directions(self, row, col, directions):
        if not directions.issubset({'N', 'E', 'S', 'W'}):
            print("Invalid directions. Must be a subset of {'N', 'E', 'S', 'W'}.")
            return

        if 0 <= row < self.rows and 0 <= col < self.cols:
            if self.grid[row][col]["type"] == "lane":
                self.grid[row][col]["directions"] = directions
            else:
                print("Cannot set directions for a parking spot.")
        else:
            print("Invalid lane coordinates.")

    # display the grid in a (semi) readable format
    def display(self):
        for row in self.grid:
            row_display = []
            for cell in row:
                if cell["type"] == "lane":
                    if len(cell["directions"]) == 4:
                        directions = "-"
                    else:
                        directions = "".join(sorted(cell["directions"]))
                    row_display.append(f" {directions} ") # allowed directions for lane
                elif cell["type"] == "parking_spot":
                    row_display.append(f" {cell['entrance']} ")  # entrance direction for parking spot
            print("".join(row_display))

# create a sample environment (shown in environment.png)
def sample_environment():
    # 7x7 environment
    parking_lot = ParkingLot(7, 7)

    # row 1: all lanes
    # row 2: parking spaces in the middle 5, accessible from the north
    for col in range(1, 6):
        parking_lot.set_parking_spot(1, col, 'N')

    # row 3: parking spaces in the middle 5, accessible from the south
    for col in range(1, 6):
        parking_lot.set_parking_spot(2, col, 'S')

    # row 4: all lanes

    # row 5: parking spaces in the middle 5, accessible from the north
    for col in range(1, 6):
        parking_lot.set_parking_spot(4, col, 'N')

    # row 6: parking spaces in the middle 5, accessible from the south
    for col in range(1, 6):
        parking_lot.set_parking_spot(5, col, 'S')

    # row 7: all lanes

    # counterclockwise movement for the lanes
    for row in range(0, 7):
        parking_lot.set_lane_directions(row, 0, {'S'})
        parking_lot.set_lane_directions(row, 6, {'N'})
    for col in range(0, 7):
        parking_lot.set_lane_directions(0, col, {'W'})
        parking_lot.set_lane_directions(6, col, {'E'})
    
    # middle lane goes left
    for col in range(1, 6):
        parking_lot.set_lane_directions(3, col, {'W'})
    
    # fix corners
    parking_lot.set_lane_directions(0, 6, {'N'})
    parking_lot.set_lane_directions(6, 0, {'S'})
    
    # middle lane junction, this cell be can accessed from two directions
    parking_lot.set_lane_directions(3, 0, {'S', 'W'})
    
    return parking_lot

if __name__ == "__main__":
    parking_lot = sample_environment()
    parking_lot.display()