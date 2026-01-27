// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "gnc_sparse_optimizer.hpp"

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <iterator>

#include "g2o/config.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/ownership.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/robust_kernel.h"

#include <boost/math/distributions/chi_squared.hpp>

namespace g2o {
using namespace std;

double Chi2inv(const double alpha, const size_t dofs) 
{
  boost::math::chi_squared_distribution<double> chi2(dofs);
  return boost::math::quantile(chi2, alpha);
}

GNCSparseOptimizer::GNCSparseOptimizer(GncLossType lossType)
    : SparseOptimizer()
{
    lossType_ = lossType;
}

GNCSparseOptimizer::GNCSparseOptimizer() 
{
    GNCSparseOptimizer(GncLossType::TLS);
}

GNCSparseOptimizer::~GNCSparseOptimizer() 
{
  release(_algorithm);
  G2OBatchStatistics::setGlobalStats(0);
}

void GNCSparseOptimizer::computeActiveErrors() 
{
  // call the callbacks in case there is something registered
  HyperGraphActionSet& actions = _graphActions[AT_COMPUTEACTIVERROR];
  if (actions.size() > 0) {
    for (HyperGraphActionSet::iterator it = actions.begin();
         it != actions.end(); ++it)
      (*(*it))(this);
  }

#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) if (_activeEdges.size() > 50)
#endif
  for (int k = 0; k < static_cast<int>(_activeEdges.size()); ++k) {
    OptimizableGraph::Edge* e = _activeEdges[k];
    e->computeError();
  }

#ifndef NDEBUG
  for (int k = 0; k < static_cast<int>(_activeEdges.size()); ++k) {
    OptimizableGraph::Edge* e = _activeEdges[k];
    bool hasNan = arrayHasNaN(e->errorData(), e->dimension());
    if (hasNan) {
      cerr << "computeActiveErrors(): found NaN in error for edge " << e
           << endl;
    }
  }
#endif
}

number_t GNCSparseOptimizer::activeChi2() const 
{
  number_t chi = 0.0;
  for (EdgeContainer::const_iterator it = _activeEdges.begin();
       it != _activeEdges.end(); ++it) {
    const OptimizableGraph::Edge* e = *it;
    chi += e->chi2();
  }
  return chi;
}

number_t GNCSparseOptimizer::activeRobustChi2() const 
{
  Vector3 rho;
  number_t chi = 0.0;
  for (EdgeContainer::const_iterator it = _activeEdges.begin();
       it != _activeEdges.end(); ++it) {
    const OptimizableGraph::Edge* e = *it;
    if (e->robustKernel()) {
      e->robustKernel()->robustify(e->chi2(), rho);
      chi += rho[0];
    } else
      chi += e->chi2();
  }
  return chi;
}


int GNCSparseOptimizer::optimize(int iterations, bool online) 
{
  
  

  return 10;
}

int GNCSparseOptimizer::defaultOptimize(bool online) 
{
  return SparseOptimizer::optimize(inner_iters_, online);
}

double GNCSparseOptimizer::initializeMu() const 
{

  return 0.0;
}

double GNCSparseOptimizer::updateMu() const 
{

  return 0.0;
}

bool GNCSparseOptimizer::checkMuConvergence(const double mu) const
{
  return false;
}

bool GNCSparseOptimizer::checkCostConvergence(const double cost, const double prev_cost) const
{
  return false;
}

bool GNCSparseOptimizer::checkKernelConvergence() const
{
  return false;
}

void GNCSparseOptimizer::updateKernels(const double mu)
{
  return;
}

void GNCSparseOptimizer::clear()
{
  egrad_.clear();
  SparseOptimizer::clear();
  return;
}


}  // namespace g2o
