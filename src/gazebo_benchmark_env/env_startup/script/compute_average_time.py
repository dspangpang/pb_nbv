def compute_average_time(file_path):
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()

        total_sum = 0
        count = 0

        for line in lines:
            try:
                # Convert line to a list of numbers
                numbers = list(map(float, line.split()))
                line_sum = sum(numbers)
                total_sum += line_sum
                count += 1
            except ValueError:
                print(f"Skipping invalid line: {line.strip()}")

        if count == 0:
            print("No valid lines found in the file.")
            return None

        average = total_sum / count
        print(f"Average of sums: {average}")
        return average

    except FileNotFoundError:
        print(f"File not found: {file_path}")
    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage
if __name__ == "__main__":
    file_path = "src/gazebo_benchmark_env/env_startup/res_data/comparison_all/figuredata/pbh-4-10-0.9,0,0_time_average.txt"  # Replace with your file path
    compute_average_time(file_path)