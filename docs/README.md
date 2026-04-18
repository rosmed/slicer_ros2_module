# Local Documentation Build

To build the documentation locally, you should create a Python virtual environment and install the required dependencies.

## Setup Instructions

1. **Create a virtual environment:**
   ```bash
   python3 -m venv venv
   ```

2. **Activate the virtual environment:**
   ```bash
   source venv/bin/activate
   ```

3. **Install the requirements:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Build the documentation:**
   ```bash
   make html
   ```

## Viewing the Results

The generated HTML files will be located in the `_build/html` directory. You can open `index.html` in your web browser to view the documentation.
