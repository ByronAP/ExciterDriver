# Contributing to ExciterDriver

First off, thank you for considering contributing! This project is open to everyone, and we welcome any help we can get. By contributing, you agree to abide by our [Code of Conduct](CODE_OF_CONDUCT.md).

Every contribution, no matter how small, is valuable. This document provides guidelines for contributing to the project to ensure a smooth and effective process for everyone involved.

## How Can I Contribute?

There are many ways you can contribute to the project:

- **Reporting Bugs:** If you find a bug, please report it.
- **Suggesting Enhancements:** Have an idea for a new feature or an improvement to an existing one? Let us know!
- **Improving Documentation:** If you find parts of the documentation unclear or incomplete, you can help improve it.
- **Submitting Code:** You can contribute code to fix bugs or add new features.

## Reporting Bugs

Before creating a bug report, please check the existing [issues](https://github.com/ByronAP/ExciterDriver/issues) to see if the bug has already been reported.

If you can't find an open issue addressing the problem, please [open a new one](https://github.com/ByronAP/ExciterDriver/issues/new). Be sure to include as much detail as possible in your report:

- **A clear and descriptive title.**
- **A detailed description of the bug.** What did you expect to happen, and what actually happened?
- **Steps to reproduce the behavior.** Provide a clear, step-by-step sequence of actions.
- **Hardware setup.** Describe the specific components you are using (Arduino model, MOSFET, etc.).
- **Software versions.** Include the version of the regulator software and the Arduino IDE you are using.
- **Screenshots or Serial Monitor logs.** If applicable, add screenshots or copy-paste relevant logs to help explain the problem.

## Suggesting Enhancements

We welcome suggestions for new features and improvements. To suggest an enhancement:

1. **Check for existing suggestions.** Search the [issues](https://github.com/ByronAP/ExciterDriver/issues) to see if your idea has already been proposed.
2. **Open a new issue.** If your idea is new, create a new issue.
3. **Provide details.** Clearly describe your proposed enhancement, explaining why it would be useful and how you envision it working.

## Your First Code Contribution

Ready to contribute code? Here’s how to set up your environment and submit your changes.

### Setting Up Your Environment

1. **Fork the repository.** Click the "Fork" button at the top right of the repository page. This creates a copy of the repository in your own GitHub account.
2. **Clone your fork.** Clone the repository to your local machine:
   ```sh
   git clone https://github.com/your-username/ExciterDriver.git
   ```
3. **Create a new branch.** It's important to create a new branch for your changes. This keeps your work separate from the main branch and makes it easier to manage.
   ```sh
   git checkout -b your-descriptive-branch-name
   ```
   For example: `git checkout -b fix-pid-overshoot` or `git checkout -b feature-temperature-comp`.

### Making Changes

1. **Write your code.** Make your changes to the source code, following the existing code style outlined in the **Style Guide** below.
2. **Add comments.** If you add a new feature or complex logic, be sure to add comments to explain what your code does.
3. **Test your changes.** Ensure that your changes work as expected and do not introduce any new bugs.
4. **Commit your changes.** Commit your changes with a clear and descriptive commit message.
   ```sh
   git add .
   git commit -m "feat: Add temperature compensation feature"
   ```
   We encourage using [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) for commit messages (e.g., `fix:`, `feat:`, `docs:`).

### Submitting a Pull Request

1. **Push your branch.** Push your changes to your fork on GitHub:
   ```sh
   git push origin your-descriptive-branch-name
   ```
2. **Open a Pull Request (PR).** Go to your fork on GitHub and click the "Compare & pull request" button.
3. **Provide a clear description.** Fill out the pull request template with a clear title and a detailed description of your changes. Explain the "why" behind your changes and link to any relevant issues.
4. **Wait for review.** Your PR will be reviewed by the project maintainers. We may ask for changes or clarification. We'll do our best to provide timely feedback.

---

## Style Guide

To maintain consistency throughout the codebase, please adhere to the following style guidelines when contributing code.

### Formatting

- **Indentation:** Use 2 spaces for indentation, not tabs.
- **Braces:** Opening braces `{` go on the same line as the statement (`if`, `for`, `while`, function definitions).
  ```cpp
  // Good
  if (condition) {
    // ...
  }

  // Bad
  if (condition)
  {
    // ...
  }
  ```
- **Spacing:** Use a single space around operators (`=`, `+`, `-`, `*`, `/`, `==`, etc.) and after commas.
  ```cpp
  // Good
  int value = someValue * 2;
  myFunction(arg1, arg2);

  // Bad
  int value=someValue*2;
  myFunction(arg1,arg2);
  ```
- **Line Length:** While not a strict rule, try to keep lines under 100 characters for better readability.

### Naming Conventions

- **Variables:** Use `camelCase` for local and global variables (e.g., `averageVoltage`, `lastError`).
- **Constants:** Use `UPPER_SNAKE_CASE` for constants defined with `const` or `#define` (e.g., `TARGET_VOLTAGE`, `MAX_PULSE_INTERVAL_MICROS`).
- **Functions:** Use `camelCase` for functions (e.g., `readVoltage`, `updateEngineState`).
- **Enums & Structs:** Use `PascalCase` for `enum` and `struct` type definitions (e.g., `EngineState`, `FaultLog`). Enum members should be `UPPER_SNAKE_CASE` (e.g., `FAULT_OVER_VOLTAGE`).
- **Pin Definitions:** Name pin constants descriptively using `camelCase` with a `Pin` suffix (e.g., `vSensePin`, `fieldControlPin`).

### Comments

- **File Header:** New `.ino` or `.h`/`.cpp` files should include a header comment explaining the purpose of the file.
- **Function Headers:** Use block comments (`/** ... */`) before complex function definitions to explain what the function does, its parameters, and what it returns.
  ```cpp
  /**
   * Blinks the error LED a specific number of times to indicate a code.
   * @param code The number of times to blink.
   */
  void blinkCode(int code) {
    // ...
  }
  ```
- **Inline Comments:** Use `//` for single-line comments to explain non-obvious or complex lines of code. Explain the *why*, not the *what*.
  ```cpp
  // Good: Explains the purpose
  // Reset the timer because the condition has cleared.
  lowVoltageTimerStart = 0;

  // Bad: Just restates the code
  // Set lowVoltageTimerStart to 0.
  lowVoltageTimerStart = 0;
  ```

Thank you for your contribution!