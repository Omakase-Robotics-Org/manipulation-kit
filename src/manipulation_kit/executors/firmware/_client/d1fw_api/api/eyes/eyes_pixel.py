from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.eyes_pixel_request import EyesPixelRequest
from ...models.eyes_pixel_response_200 import EyesPixelResponse200
from ...types import Response


def _get_kwargs(
    *,
    body: EyesPixelRequest,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/eyes/pixel",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | EyesPixelResponse200 | None:
    if response.status_code == 200:
        response_200 = EyesPixelResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ErrorEnvelope | EyesPixelResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: EyesPixelRequest,
) -> Response[ErrorEnvelope | EyesPixelResponse200]:
    """Place one pixel by freezing a running-light window

     The controller has no addressable single-pixel write, so the placement starts a running-light window
    and stops it on the wanted index. The report says where the stop actually landed: a `stop_after_ms`
    more than half a step from `target_ms` means the pixel may be one index off.

    Args:
        body (EyesPixelRequest): One frozen-window single-pixel placement request.

            `index` is the LED index as the eye controller counts the strip
            (`0..=17`), `target` names one eye (`left` or `right`; `both` is refused
            because the two strips do not run a `BOTH`-addressed window in phase), and
            `delay_ms` is the window speed in milliseconds per index, defaulting to
            the measured 100 ms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | EyesPixelResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: EyesPixelRequest,
) -> ErrorEnvelope | EyesPixelResponse200 | None:
    """Place one pixel by freezing a running-light window

     The controller has no addressable single-pixel write, so the placement starts a running-light window
    and stops it on the wanted index. The report says where the stop actually landed: a `stop_after_ms`
    more than half a step from `target_ms` means the pixel may be one index off.

    Args:
        body (EyesPixelRequest): One frozen-window single-pixel placement request.

            `index` is the LED index as the eye controller counts the strip
            (`0..=17`), `target` names one eye (`left` or `right`; `both` is refused
            because the two strips do not run a `BOTH`-addressed window in phase), and
            `delay_ms` is the window speed in milliseconds per index, defaulting to
            the measured 100 ms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | EyesPixelResponse200
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: EyesPixelRequest,
) -> Response[ErrorEnvelope | EyesPixelResponse200]:
    """Place one pixel by freezing a running-light window

     The controller has no addressable single-pixel write, so the placement starts a running-light window
    and stops it on the wanted index. The report says where the stop actually landed: a `stop_after_ms`
    more than half a step from `target_ms` means the pixel may be one index off.

    Args:
        body (EyesPixelRequest): One frozen-window single-pixel placement request.

            `index` is the LED index as the eye controller counts the strip
            (`0..=17`), `target` names one eye (`left` or `right`; `both` is refused
            because the two strips do not run a `BOTH`-addressed window in phase), and
            `delay_ms` is the window speed in milliseconds per index, defaulting to
            the measured 100 ms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | EyesPixelResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: EyesPixelRequest,
) -> ErrorEnvelope | EyesPixelResponse200 | None:
    """Place one pixel by freezing a running-light window

     The controller has no addressable single-pixel write, so the placement starts a running-light window
    and stops it on the wanted index. The report says where the stop actually landed: a `stop_after_ms`
    more than half a step from `target_ms` means the pixel may be one index off.

    Args:
        body (EyesPixelRequest): One frozen-window single-pixel placement request.

            `index` is the LED index as the eye controller counts the strip
            (`0..=17`), `target` names one eye (`left` or `right`; `both` is refused
            because the two strips do not run a `BOTH`-addressed window in phase), and
            `delay_ms` is the window speed in milliseconds per index, defaulting to
            the measured 100 ms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | EyesPixelResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
