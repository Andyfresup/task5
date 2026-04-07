#!/bin/bash
# CADE Project Automatic Setup Script

set -e  # Exit immediately on error

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  🤖 CADE Environment Setup Script                         ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if conda is installed
if ! command -v conda &> /dev/null; then
    echo -e "${RED}❌ Conda not detected${NC}"
    echo ""
    echo "Please install Miniforge first:"
    echo "  1. Download: wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh"
    echo "  2. Install: bash Miniforge3-Linux-x86_64.sh"
    echo "  3. Restart terminal or run: source ~/.bashrc"
    echo ""
    exit 1
fi

echo -e "${GREEN}✓${NC} Conda detected: $(conda --version)"
echo ""

# Check if environment already exists
if conda env list | grep -q "^cade "; then
    echo -e "${YELLOW}⚠️  Environment 'cade' already exists${NC}"
    read -p "Remove and rebuild? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "🗑️  Removing old environment..."
        conda env remove -n cade -y
    else
        echo "Using existing environment"
        ENV_EXISTS=true
    fi
fi

# Create conda environment
if [ "$ENV_EXISTS" != "true" ]; then
    echo "📦 Creating conda environment 'cade' (Python 3.11)..."
    conda create -n cade python=3.11 -y
    echo -e "${GREEN}✓${NC} Environment created"
fi

echo ""

# Activate environment
echo "🔄 Activating environment..."
source $(conda info --base)/etc/profile.d/conda.sh
conda activate cade

# Verify Python
echo -e "${GREEN}✓${NC} Python: $(python --version)"
echo -e "${GREEN}✓${NC} Python path: $(which python)"
echo ""

# Install dependencies
echo "📥 Installing dependencies..."
pip install -r requirements.txt
echo -e "${GREEN}✓${NC} Dependencies installed"
echo ""

# Setup configuration file
if [ ! -f src/config_local.py ]; then
    echo "📝 Creating configuration file..."
    cp src/config_local.example.py src/config_local.py
    echo -e "${GREEN}✓${NC} Created src/config_local.py"
    echo ""
    echo -e "${YELLOW}⚠️  Important:${NC}"
    echo "   Please edit src/config_local.py and add your API key"
    echo "   Or install Ollama to use local models"
    echo ""
else
    echo -e "${GREEN}✓${NC} src/config_local.py already exists"
    echo ""
fi

# Complete
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  ✅ Setup Complete!                                       ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "Next steps:"
echo "  1. Activate environment: conda activate cade"
echo "  2. Configure API: nano src/config_local.py"
echo "  3. Run demo: python src/main.py --demo"
echo "  4. Interactive mode: python src/main.py"
echo ""
