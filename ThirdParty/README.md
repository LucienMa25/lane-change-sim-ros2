# ThirdParty — HSL 稀疏求解器源码目录

本仓库**不包含** HSL（MA27）源码，因其受 [STFC](https://licences.stfc.ac.uk/product/coin-hsl-archive)
许可证约束，需自行注册下载（学术/商业均免费）。

## 放置方法

1. 注册并下载 `coinhsl-archive`（本项目基于 **2021.05.05** 版本验证，目录名需保持一致）
2. 解压到本目录，最终路径应为：

   ```text
   ThirdParty/coinhsl-archive-2021.05.05/
   ```

3. **补上脚本执行权限**（压缩包经某些渠道传输会丢失权限位，否则编译报 `configure: Permission denied`）：

   ```bash
   chmod +x coinhsl-archive-2021.05.05/{configure,install-sh,missing,compile,config.guess,config.sub,depcomp,ltmain.sh}
   ```

4. 正常执行 `colcon build` 即可，CMake 会自动检测并编译 HSL（`WITH_HSL` 默认 ON）。

## 没有 HSL 也能编译吗？

可以。将 CasADi/IPOPT 的回退线性求解器换为开源 MUMPS 即可正常工作，仅求解速度较慢
（MA27 在 banded 结构问题上通常快 2–5 倍）。注意：当前 `CMakeLists.txt` 默认
`-DWITH_HSL=ON` 并指向本目录源码，若不下发 HSL，请将 ExternalProject 参数中的
`-DWITH_HSL=ON` / `-DWITH_BUILD_HSL=ON` 调整为 OFF 后再构建。
