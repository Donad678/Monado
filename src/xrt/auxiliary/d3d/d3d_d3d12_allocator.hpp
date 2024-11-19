// Copyright 2020-2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Higher-level D3D12-backed image buffer allocation routine.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Fernando Velazquez Innella <finnella@magicleap.com>
 * @ingroup aux_d3d
 */

#pragma once

#include "xrt/xrt_compositor.h"

#include <Unknwn.h>
#include <d3d12.h>
#include <wil/com.h>
#include <wil/resource.h>

#include <vector>


namespace xrt::auxiliary::d3d::d3d12 {

/*!
 * Swapchain creation info in D3D12 terms
 */
struct d3d12_swapchain_create_info
{
	size_t image_count;
	D3D12_RESOURCE_DESC desc;
	D3D12_RESOURCE_STATES initial_resource_state;
	D3D12_HEAP_PROPERTIES heap;
	D3D12_HEAP_FLAGS heap_flags;
};

/**
 * Validate and convert an @ref xrt_swapchain_create_info into an @ref d3d12_swapchain_create_info.
 * This is the form @ref allocateSharedImages needs to create the swapchain images.
 *
 * @param xsci Swapchain create info: note that the format is assumed to be a DXGI_FORMAT (conversion to typeless is
 * automatic)
 * @param image_count The number of images intended to be created. This will be validated against CREATE_STATIC_IMAGE.
 * @param[out] out_create_info The create info in D3D12 terms. If an error is returned, some fields may be overwritten.
 *
 * @return xrt_result_t, one of:
 * - @ref XRT_SUCCESS
 * - @ref XRT_ERROR_ALLOCATION
 * - @ref XRT_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED
 * - @ref XRT_ERROR_SWAPCHAIN_FLAG_VALID_BUT_UNSUPPORTED
 */
xrt_result_t
convertCreateInfoToD3d12(const xrt_swapchain_create_info &xsci,
                         size_t image_count, d3d12_swapchain_create_info& out_create_info);

/**
 * Allocate images (ID3D12Resource) that have a corresponding native handle.
 *
 * @param device A D3D12 device to allocate with.
 * @param dsci Swapchain create info (in D3D12 terms). Not validated here; the only error is from the driver.
 * If you have an @ref xrt_swapchain_create_info, use @ref convertCreateInfoToD3d12 to validate and convert.
 * @param[out] out_images A vector that will be cleared and populated with the images.
 * @param[out] out_handles A vector that will be cleared and populated with the corresponding native handles.
 *
 * @return xrt_result_t, one of:
 * - @ref XRT_SUCCESS
 * - @ref XRT_ERROR_ALLOCATION
 */
xrt_result_t
allocateSharedImages(ID3D12Device &device,
                     const d3d12_swapchain_create_info &dsci,
                     std::vector<wil::com_ptr<ID3D12Resource>> &out_images,
                     std::vector<wil::unique_handle> &out_handles);

}; // namespace xrt::auxiliary::d3d::d3d12
