file_name = 'motor_coords'
 
def read_coords(file_name):
    numbers = [0,0]
    try:
        with open(file_name, 'r') as file:
            line = file.read().strip()
            numbers = [int(num) for num in line.split(',')]
    except FileNotFoundError:
        print("Coordinate file not found")
    except ValueError:
        print("Not valid coordinates")
    return numbers

def write_coords(file_name,numbers):
    with open(file_name, 'w') as file:
        file.write(', '.join(map(str,numbers)))

def main():
    numbers = read_coords(file_name)
    print(numbers)
    new_numbers = [num + 12 for num in numbers]
    write_coords(file_name,new_numbers)

if __name__  == '__main__':
    main()