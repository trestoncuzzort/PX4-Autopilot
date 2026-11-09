/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef MODULE_NAME
#define MODULE_NAME "lockstep"
#endif

#include <lockstep_scheduler/lockstep_components.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <limits.h>

LockstepComponents::LockstepComponents(bool no_cleanup_on_destroy)
	: _no_cleanup_on_destroy(no_cleanup_on_destroy)
{
}

LockstepComponents::~LockstepComponents()
{
	// Trying to destroy a condition variable with threads currently blocked on it results in undefined behavior.
	// Therefore we allow the caller not to cleanup and let the OS take care of that.
	if (!_no_cleanup_on_destroy) {
		pthread_cond_destroy(&_condition);
		pthread_mutex_destroy(&_mutex);
	}
}

int LockstepComponents::register_component()
{
	pthread_mutex_lock(&_mutex);

	for (int component = 0; component < (int)sizeof(int) * CHAR_BIT - 1; ++component) {
		if (!(_components_used_bitset & (1 << component))) {
			_components_used_bitset |= 1 << component;
			pthread_mutex_unlock(&_mutex);
			PX4_DEBUG("got lockstep component %i", component);
			return 1 << component;
		}
	}

	pthread_mutex_unlock(&_mutex);
	PX4_ERR("No more components left");
	return 0;
}

void LockstepComponents::unregister_component(int component)
{
	if (component <= 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);
	_components_progress_bitset &= ~component;
	_components_used_bitset &= ~component;

	if (_components_progress_bitset == _components_used_bitset) {
		pthread_cond_signal(&_condition);
	}

	pthread_mutex_unlock(&_mutex);
}

void LockstepComponents::lockstep_progress(int component)
{
	if (component <= 0) {
		return;
	}

	pthread_mutex_lock(&_mutex);
	_components_progress_bitset |= component;

	if (_components_progress_bitset == _components_used_bitset) {
		pthread_cond_signal(&_condition);
	}

	pthread_mutex_unlock(&_mutex);
}

void LockstepComponents::wait_for_components()
{
	pthread_mutex_lock(&_mutex);

	while (_components_progress_bitset != _components_used_bitset) {
		pthread_cond_wait(&_condition, &_mutex);
	}

	// Consume progress here, so a completed idle interval cannot release a later cycle with new work.
	_components_progress_bitset = 0;
	pthread_mutex_unlock(&_mutex);
}
