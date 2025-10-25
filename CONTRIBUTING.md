# Contributing to DroneKit

Thank you for your interest in contributing to DroneKit! This document provides guidelines and instructions for contributing.

## Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Help others learn and grow
- Report unacceptable behavior

## How to Contribute

### Reporting Bugs

When reporting bugs, please include:
- A clear, descriptive title
- Detailed steps to reproduce the issue
- Expected behavior vs actual behavior
- Your environment (Python version, OS, etc.)
- Any relevant error messages or logs

### Suggesting Enhancements

Enhancement suggestions should include:
- Clear description of the feature
- Use cases and benefits
- Potential implementation approach
- Any breaking changes or compatibility concerns

### Pull Requests

1. **Fork the repository** and create your branch from `main`
2. **Make your changes** following the code style guidelines
3. **Add tests** for any new functionality
4. **Update documentation** if needed
5. **Ensure tests pass** by running `pytest`
6. **Submit a pull request** with a clear description

## Development Setup

1. Clone your fork:
```bash
git clone https://github.com/YOUR_USERNAME/dronekit.git
cd dronekit
```

2. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install in development mode:
```bash
pip install -e ".[dev]"
```

## Code Style

- Follow PEP 8 style guidelines
- Use meaningful variable and function names
- Add docstrings to all public functions and classes
- Keep functions focused and single-purpose
- Maximum line length: 100 characters

### Type Hints

Use type hints for function arguments and return values:
```python
def connect(connection_string: str, wait_ready: bool = False) -> Vehicle:
    pass
```

### Docstrings

Use Google-style docstrings:
```python
def arm(self, wait: bool = False, timeout: float = 30.0):
    """
    Arm the vehicle.
    
    Args:
        wait: Wait for arming to complete
        timeout: Timeout in seconds
    
    Raises:
        ArmingError: If arming fails
    
    Example:
        >>> vehicle.arm(wait=True)
    """
    pass
```

## Testing

### Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=dronekit

# Run specific test file
pytest tests/test_vehicle.py

# Run with verbose output
pytest -v
```

### Writing Tests

- Place tests in the `tests/` directory
- Name test files `test_*.py`
- Name test functions `test_*`
- Use descriptive test names
- Test both success and failure cases
- Mock external dependencies (MAVLink connections)

Example:
```python
def test_vehicle_armed_property():
    """Test that armed property returns correct state."""
    vehicle = Vehicle("127.0.0.1:14550")
    vehicle._connected = True
    vehicle._armed = True
    assert vehicle.armed is True
```

## Code Review Process

1. All submissions require review
2. Maintainers will review code for:
   - Correctness
   - Code quality
   - Test coverage
   - Documentation
3. Address review feedback
4. Once approved, a maintainer will merge

## Commit Messages

Write clear commit messages:
- Use present tense ("Add feature" not "Added feature")
- Use imperative mood ("Move cursor to..." not "Moves cursor to...")
- First line should be 50 characters or less
- Reference issues and pull requests when relevant

Example:
```
Add support for custom MAVLink messages

- Implement custom message handler
- Add tests for custom messages
- Update documentation

Fixes #123
```

## Documentation

Update documentation when:
- Adding new features
- Changing public APIs
- Fixing bugs that affect usage
- Adding examples

Documentation locations:
- `README.md` - Main documentation
- Docstrings - API documentation
- `examples/` - Usage examples

## Release Process

Maintainers handle releases:
1. Update version in `setup.py`
2. Update CHANGELOG
3. Create git tag
4. Build and publish to PyPI

## Questions?

If you have questions about contributing, feel free to:
- Open an issue
- Ask in pull request comments
- Contact maintainers

Thank you for contributing to DroneKit!
