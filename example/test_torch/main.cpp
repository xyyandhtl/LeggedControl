#include <torch/script.h>
#include <torch/torch.h>
#include <iostream>

int main() {

    // 加载 TorchScript 模型
    torch::jit::script::Module module = torch::jit::load("/home/lenovo/Documents/shared/policy_amp.pt");
    module.to(torch::kCUDA);

    for (const auto& param : module.named_parameters()) {
        std::cout << param.name << ": " << param.value.sizes() << std::endl;
    }

    int num_env = 1;
    int obs_dim = 270;

    // 创建输入 Tensor
    torch::Tensor input = torch::randn({num_env, obs_dim}, torch::dtype(torch::kFloat32)).to(torch::kCUDA);

    // 封装为 IValue
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input);

    // 前向推理
    at::Tensor output = module.forward(inputs).toTensor();

    // 打印形状
    std::cout << "Input shape : " << input.sizes() << "\n";
    std::cout << "Output shape: " << output.sizes() << "\n";
    std::cout << "Sample output:\n"
                << output.index({torch::indexing::Slice(0, 2),
                                torch::indexing::Slice(0, 5)})
                << "\n";

    return 0;
}
