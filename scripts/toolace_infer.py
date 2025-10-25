from transformers import AutoModelForCausalLM, AutoTokenizer

def load_qwen_model() -> tuple[AutoModelForCausalLM, AutoTokenizer]:
    """Load the Qwen model and tokenizer."""
    model_name = "Team-ACE/ToolACE-2-Llama-3.1-8B"
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    model = AutoModelForCausalLM.from_pretrained(model_name)
    return model, tokenizer