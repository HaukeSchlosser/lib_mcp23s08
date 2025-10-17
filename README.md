# MCP23S08 SPI Library

A very lightweight C/C++ library for interfacing with the **MCP23S08** (8-bit I/O expander) via SPI.

---

### Installation

#### Clone the Repository
```sh
git clone https://github.com/HaukeSchlosser/lib_mcp23s08.git
cd lib_mcp23s08
```

#### Compile
```sh
make
```

#### Install
```sh
sudo make install
```

---

### Usage

#### Compile Program
```sh
gcc  <program_name>.c -L/usr/local/lib -lmcp23s08 -o <program_name>
```

---

### Clean-Up

#### Uninstall Library
```sh
sudo make uninstall
```

#### Clean-Up build files
```sh
make clean
```

#### Clean Everything
```sh
make cleanall
```

---

### License

This library is licensed under the GNU License.

---

### Author

Developed by Hauke Schlosser. Contributions are welcome!
