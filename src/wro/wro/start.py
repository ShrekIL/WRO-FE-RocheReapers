import requests # Using the 'requests' library for simplicity. Install with 'pip install requests'
import json


server_url="http://localhost:8000/is_ready"

def is_ready():
    """
    Queries the /is_ready endpoint of the GPIO web server and returns its status.

    Args:
        server_url (str): The URL of the /is_ready endpoint.
                          Defaults to "http://localhost:8000/is_ready".

    Returns:
        bool: True if the server reports 'ready: true', False otherwise (including errors).
    """
    try:
        # Make a GET request to the server
        response = requests.get(server_url, timeout=5) # Added a 5-second timeout

        # Check if the request was successful (status code 200)
        if response.status_code == 200:
            try:
                # Parse the JSON response
                data = response.json()
                # Return the boolean value of the 'ready' key
                return bool(data.get('ready', False)) # Default to False if 'ready' key is missing
            except json.JSONDecodeError:
                print(f"Error: Could not decode JSON response from {server_url}")
                return False
            except KeyError:
                print(f"Error: 'ready' key not found in JSON response from {server_url}")
                return False
        else:
            print(f"Error: Server returned status code {response.status_code} from {server_url}")
            return False
    except requests.exceptions.ConnectionError:
        print(f"Error: Could not connect to the server at {server_url}. Is it running?")
        return False
    except requests.exceptions.Timeout:
        print(f"Error: Request to {server_url} timed out.")
        return False
    except requests.exceptions.RequestException as e:
        print(f"An unexpected error occurred while requesting {server_url}: {e}")
        return False
    
if __name__ == "__main__":
    print(is_ready())
    